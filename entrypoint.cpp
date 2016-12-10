/*
#==========================================================================================
# + + +   This Software is released under the "Simplified BSD License"  + + +
# Copyright 2014 F4GKR Sylvain AZARIAN . All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, are
#permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright notice, this list of
#	  conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright notice, this list
#	  of conditions and the following disclaimer in the documentation and/or other materials
#	  provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY Sylvain AZARIAN F4GKR ``AS IS'' AND ANY EXPRESS OR IMPLIED
#WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
#FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Sylvain AZARIAN OR
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#The views and conclusions contained in the software and documentation are those of the
#authors and should not be interpreted as representing official policies, either expressed
#or implied, of Sylvain AZARIAN F4GKR.
#
# Adds AirSpy capability to SDRNode
#==========================================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "AirSpy.h"
#include "jansson/jansson.h"

#include "entrypoint.h"
#define DEBUG_DRIVER (0)
#define STAGES_COUNT (3)


char *driver_name ;
void* acquisition_thread( void *params ) ;

typedef struct __attribute__ ((__packed__)) _sCplx
{
    float re;
    float im;
} TYPECPX;

struct t_sample_rates {
    unsigned int *sample_rates ;
    int enum_length ;
    int preffered_sr_index ;
};

// this structure stores the device state
struct t_rx_device {
    struct airspy_device* device ;
    char *device_name ;
    char *device_serial_number ;

    struct t_sample_rates* rates;
    int current_sample_rate ;

    int64_t min_frq_hz ; // minimal frequency for this device
    int64_t max_frq_hz ; // maximal frequency for this device
    int64_t center_frq_hz ; // currently set frequency

    float gainv[STAGES_COUNT] ;

    char *uuid ;
    bool running ;
    bool acq_stop ;
    sem_t mutex;

    pthread_t receive_thread ;
    struct ext_Context ext_context ;

    struct t_rx_device *prev = NULL ;
    struct t_rx_device *next = NULL ;
};

int device_count ;
char *stage_name[STAGES_COUNT] ;
char *stage_unit ;

struct t_rx_device *rx;
struct t_rx_device **rxtab;

json_t *root_json ;
_tlogFun* sdrNode_LogFunction ;
_pushSamplesFun *acqCbFunction ;

#ifdef _WIN64
#include <windows.h>
// Win  DLL Main entry
BOOL WINAPI DllMain( HINSTANCE hInstance, DWORD dwReason, LPVOID *lpvReserved ) {
    return( TRUE ) ;
}
#endif

void log( int device_id, int level, char *msg ) {
    if( sdrNode_LogFunction != NULL ) {
        (*sdrNode_LogFunction)(rx[device_id].uuid,level,msg);
        return ;
    }
    printf("Trace:%s\n", msg );
}



/*
 * First function called by SDRNode - must return 0 if hardware is not present or problem
 */
/**
 * @brief initLibrary is called when the DLL is loaded, only for the first instance of the devices (when the getBoardCount() function returns
 *        more than 1)
 * @param json_init_params a JSOn structure to pass parameters from scripting to drivers
 * @param ptr pointer to function for logging
 * @param acqCb pointer to RF IQ processing function
 * @return
 */
LIBRARY_API int initLibrary(char *json_init_params,
                            _tlogFun* ptr,
                            _pushSamplesFun *acqCb ) {
    json_error_t error;
    root_json = NULL ;
    struct t_rx_device *tmp ;
    int rc ;

    sdrNode_LogFunction = ptr ;
    acqCbFunction = acqCb ;

    if( json_init_params != NULL ) {
        root_json = json_loads(json_init_params, 0, &error);

    }
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);

    driver_name = (char *)malloc( 100*sizeof(char));
    snprintf(driver_name,100,"AirSpy");

    // Step 1 : count how many devices we have
    rc = airspy_init() ;
    if( rc != AIRSPY_SUCCESS ) {
        airspy_exit();
        return(0);
    }

    struct airspy_device* device ;
    struct t_rx_device *prev = NULL ;

    device_count = 0 ;
    while( airspy_open( &device ) == AIRSPY_SUCCESS ) {
          tmp = (struct t_rx_device *)malloc(sizeof(struct t_rx_device));
          if( tmp == NULL )
              break ;

          if( device_count == 0 ) {
              rx = tmp ;
          }

          device_count++ ;
          tmp->device = device ;
          tmp->uuid = NULL ;
          tmp->running = false ;
          tmp->acq_stop = false ;
          tmp->next = NULL ;
          tmp->prev = NULL ;
          sem_init(&tmp->mutex, 0, 0);


          tmp->min_frq_hz = 24000000 ;
          tmp->max_frq_hz = 1900000000ul ;
          tmp->center_frq_hz = tmp->min_frq_hz + 1e6 ; // arbitrary startup freq

          // alloc name - TODO : check if Spy, Spy2, Mini etc.
          tmp->device_name = (char *)malloc( 64 *sizeof(char));
          sprintf( tmp->device_name, "AirSpy");

          tmp->device_serial_number = (char *)malloc( 64 *sizeof(char));

          airspy_read_partid_serialno_t serial;
          airspy_board_partid_serialno_read( device, &serial );

          sprintf( tmp->device_serial_number, "0x%08X%08X",
                   serial.serial_no[2],
                   serial.serial_no[3]);

          uint32_t num_rates;
          airspy_get_samplerates(device, &num_rates, 0);
          uint32_t *samplerates = (uint32_t *) malloc(num_rates * sizeof(uint32_t));
          airspy_get_samplerates( device, samplerates, num_rates);

          tmp->rates = (struct t_sample_rates*)malloc( sizeof(struct t_sample_rates));
          tmp->rates->enum_length = num_rates ;
          tmp->rates->sample_rates = (unsigned int *)malloc( tmp->rates->enum_length * sizeof( unsigned int )) ;
          for( uint32_t i=0 ; i < num_rates ; i++ ) {
              tmp->rates->sample_rates[i] = (unsigned int)samplerates[i];
          }
          tmp->rates->preffered_sr_index = 0 ;
          tmp->current_sample_rate = tmp->rates->sample_rates[tmp->rates->preffered_sr_index] ;

          // configure device
          airspy_set_samplerate( device, tmp->current_sample_rate );
          airspy_set_freq( device, tmp->center_frq_hz);
          airspy_set_packing( device, 1 ); // use packing
          // create waiting thread for this device
          pthread_create(&tmp->receive_thread, NULL, acquisition_thread, tmp );

          for( int i=0 ; i < STAGES_COUNT ; i++ ) {
              tmp->gainv[i] = 6 ;
          }
          airspy_set_lna_gain( device, 6 );
          airspy_set_vga_gain( device, 6 );
          airspy_set_mixer_gain( device, 6);

          tmp->ext_context.ctx_version = 0 ;

          if( prev != NULL ) {
              prev->next = tmp ;
          }
          tmp->prev = prev ;
          prev = tmp ;
    }
    // go through linked list and build table
    if( device_count == 0 ) {
        airspy_exit();
        return(0);
    }

    rxtab = (struct t_rx_device **)malloc( device_count * sizeof(struct t_rx_device));
    tmp = rx ;
    for( int i=0 ; (i < device_count) && (tmp!=NULL); i++ ) {
         rxtab[i] = tmp ;
         tmp = tmp->next ;
    }

    // set names for stages
    stage_name[0] = (char *)malloc( 10*sizeof(char));
    snprintf( stage_name[0],10,"LNA");
    stage_name[1] = (char *)malloc( 10*sizeof(char));
    snprintf( stage_name[1],10,"MIXER");
    stage_name[2] = (char *)malloc( 10*sizeof(char));
    snprintf( stage_name[2],10,"VGA");

    stage_unit = (char *)malloc( 10*sizeof(char));
    snprintf( stage_unit,10,"dB");

    return(RC_OK);
}


/**
 * @brief setBoardUUID this function is called by SDRNode to assign a unique ID to each device managed by the driver
 * @param device_id [0..getBoardCount()[
 * @param uuid the unique ID
 * @return
 */
LIBRARY_API int setBoardUUID( int device_id, char *uuid ) {
    int len = 0 ;

    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%s)\n", __func__, device_id, uuid );

    if( uuid == NULL ) {
        return(RC_NOK);
    }
    if( device_id >= device_count )
        return(RC_NOK);

    len = strlen(uuid);
    if( rxtab[device_id]->uuid != NULL ) {
        free( rxtab[device_id]->uuid );
    }
    rxtab[device_id]->uuid = (char *)malloc( len * sizeof(char));
    strcpy( rxtab[device_id]->uuid, uuid);
    return(RC_OK);
}

/**
 * @brief getHardwareName called by SDRNode to retrieve the name for the nth device
 * @param device_id [0..getBoardCount()[
 * @return a string with the hardware name, this name is listed in the 'devices' admin page and appears 'as is' in the scripts
 */
LIBRARY_API char *getHardwareName(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(NULL);
    struct t_rx_device *dev = rxtab[device_id] ;
    return( dev->device_name );
}

/**
 * @brief getBoardCount called by SDRNode to retrieve the number of different boards managed by the driver
 * @return the number of devices managed by the driver
 */
LIBRARY_API int getBoardCount() {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    return(device_count);
}

/**
 * @brief getPossibleSampleRateCount called to know how many sample rates are available. Used to fill the select zone in admin
 * @param device_id
 * @return sample rate in Hz
 */
LIBRARY_API int getPossibleSampleRateCount(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = rxtab[device_id] ;
    return( dev->rates->enum_length );
}

/**
 * @brief getPossibleSampleRateValue
 * @param device_id
 * @param index
 * @return
 */
LIBRARY_API unsigned int getPossibleSampleRateValue(int device_id, int index) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, index );
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = rxtab[device_id] ;

    struct t_sample_rates* rates = dev->rates ;
    if( index > rates->enum_length )
        return(0);

    return( rates->sample_rates[index] );
}

LIBRARY_API unsigned int getPrefferedSampleRateValue(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = rxtab[device_id] ;
    struct t_sample_rates* rates = dev->rates ;
    int index = rates->preffered_sr_index ;
    return( rates->sample_rates[index] );
}
//-------------------------------------------------------------------
LIBRARY_API int64_t getMin_HWRx_CenterFreq(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = rxtab[device_id] ;
    return( dev->min_frq_hz ) ;
}

LIBRARY_API int64_t getMax_HWRx_CenterFreq(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = rxtab[device_id] ;
    return( dev->max_frq_hz ) ;
}

//-------------------------------------------------------------------
// Gain management
// devices have stages (LNA, VGA, IF...) . Each stage has its own gain
// range, its own name and its own unit.
// each stage can be 'continuous gain' or 'discrete' (on/off for example)
//-------------------------------------------------------------------
LIBRARY_API int getRxGainStageCount(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    return(3);
}

LIBRARY_API char* getRxGainStageName( int device_id, int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    if( stage >= STAGES_COUNT ) {
        stage = 0 ;
    }
    return( stage_name[stage] );
}

LIBRARY_API char* getRxGainStageUnitName( int device_id, int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    return( stage_unit );
}

LIBRARY_API int getRxGainStageType( int device_id, int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    // continuous value
    return(0);
}

LIBRARY_API float getMinGainValue(int device_id,int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );

    return(0) ;
}

LIBRARY_API float getMaxGainValue(int device_id,int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    return(15) ;
}

LIBRARY_API int getGainDiscreteValuesCount( int device_id, int stage ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage);
    return(0);
}

LIBRARY_API float getGainDiscreteValue( int device_id, int stage, int index ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d, %d,%d)\n", __func__, device_id, stage, index);
    return(0);
}

/**
 * @brief getSerialNumber returns the (unique for this hardware name) serial number. Serial numbers are useful to manage more than one unit
 * @param device_id
 * @return
 */
LIBRARY_API char* getSerialNumber( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);
    struct t_rx_device *dev = rxtab[device_id] ;
    return( dev->device_serial_number );
}

//----------------------------------------------------------------------------------
// Manage acquisition
// SDRNode calls 'prepareRxEngine(device)' to ask for the start of acquisition
// Then, the driver shall call the '_pushSamplesFun' function passed at initLibrary( ., ., _pushSamplesFun* fun , ...)
// when the driver shall stop, SDRNode calls finalizeRXEngine()

/**
 * @brief prepareRXEngine trig on the acquisition process for the device
 * @param device_id
 * @return RC_OK if streaming has started, RC_NOK otherwise
 */
LIBRARY_API int prepareRXEngine( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);

    // here we keep it simple, just fire the relevant mutex
    struct t_rx_device *dev = rxtab[device_id] ;
    dev->acq_stop = false ;
    sem_post(&dev->mutex);

    return(RC_OK);
}

/**
 * @brief finalizeRXEngine stops the acquisition process
 * @param device_id
 * @return
 */
LIBRARY_API int finalizeRXEngine( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);

    struct t_rx_device *dev = rxtab[device_id] ;
    if( airspy_is_streaming( dev->device )!= AIRSPY_TRUE ) {
        return(RC_OK);
    }
    dev->acq_stop = true ;
    int rc = airspy_stop_rx(dev->device);
    if( rc == AIRSPY_SUCCESS ) {
        return(RC_OK);
    }
    return(RC_NOK);
}

/**
 * @brief setRxSampleRate configures the sample rate for the device (in Hz). Can be different from the enum given by getXXXSampleRate
 * @param device_id
 * @param sample_rate
 * @return
 */
LIBRARY_API int setRxSampleRate( int device_id , int sample_rate) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id,sample_rate);
    if( device_id >= device_count )
        return(RC_NOK);

    struct t_rx_device *dev = rxtab[device_id] ;
    if( sample_rate == dev->current_sample_rate ) {
        return(RC_OK);
    }

    int rc = airspy_set_samplerate( dev->device, sample_rate );
    if( rc == AIRSPY_SUCCESS ) {
        dev->current_sample_rate = sample_rate ;
        dev->ext_context.sample_rate = sample_rate ;
        dev->ext_context.ctx_version++ ;
        return(RC_OK);
    }
    return(RC_NOK);
}

/**
 * @brief getActualRxSampleRate called to know what is the actual sampling rate (hz) for the given device
 * @param device_id
 * @return
 */
LIBRARY_API int getActualRxSampleRate( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);
    struct t_rx_device *dev = rxtab[device_id] ;
    return(dev->current_sample_rate);
}

/**
 * @brief setRxCenterFreq tunes device to frq_hz (center frequency)
 * @param device_id
 * @param frq_hz
 * @return
 */
LIBRARY_API int setRxCenterFreq( int device_id, int64_t frq_hz ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%ld)\n", __func__, device_id, (long)frq_hz);
    if( DEBUG_DRIVER ) fflush(stderr);
    if( device_id >= device_count )
        return(RC_NOK);

    struct t_rx_device *dev = rxtab[device_id] ;
    int rc = airspy_set_freq( dev->device, (uint32_t)frq_hz  );
    if( rc == AIRSPY_SUCCESS ) {
        dev->center_frq_hz = frq_hz ;
        dev->ext_context.center_freq = frq_hz ;
        dev->ext_context.ctx_version++ ;
        return(RC_OK);
    }
    return(RC_NOK);
}

/**
 * @brief getRxCenterFreq retrieve the current center frequency for the device
 * @param device_id
 * @return
 */
LIBRARY_API int64_t getRxCenterFreq( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);

    struct t_rx_device *dev = rxtab[device_id] ;
    return( dev->center_frq_hz ) ;
}

/**
 * @brief setRxGain sets the current gain
 * @param device_id
 * @param stage_id
 * @param gain_value
 * @return
 */
LIBRARY_API int setRxGain( int device_id, int stage_id, float gain_value ) {
    int rc ;
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d,%f)\n", __func__, device_id,stage_id,gain_value);
    if( device_id >= device_count )
        return(RC_NOK);
    if( stage_id >= STAGES_COUNT )
        return(RC_NOK);

    struct t_rx_device *dev = rxtab[device_id] ;
    airspy_device *spydev = dev->device ;
    uint8_t m_Gain = (uint8_t)gain_value ;
    switch( stage_id ) {
        case 0: rc = airspy_set_lna_gain( spydev, m_Gain);
            break ;
        case 1: rc = airspy_set_vga_gain( spydev, m_Gain);
            break ;

        default: rc = airspy_set_mixer_gain( spydev, m_Gain);
            break ;
    }

    if( rc == AIRSPY_SUCCESS ) {
        dev->gainv[stage_id] = gain_value ;
        return(RC_OK);
    }
    return(RC_NOK);
}

/**
 * @brief getRxGainValue reads the current gain value
 * @param device_id
 * @param stage_id
 * @return
 */
LIBRARY_API float getRxGainValue( int device_id , int stage_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id,stage_id);

    if( device_id >= device_count )
        return(RC_NOK);
    if( stage_id >= STAGES_COUNT )
        return(RC_NOK);

    struct t_rx_device *dev = rxtab[device_id] ;
    return( dev->gainv[stage_id]) ;
}

LIBRARY_API bool setAutoGainMode( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    return(false);
}

//-----------------------------------------------------------------------------------------
// One thread is started by device, and each sample frame calls sdr_callback() with a block
// of IQ samples as bytes.


/**
 * @brief sdr_callback called by driver.
 */
int sdr_callback( airspy_transfer* transfer ) {
    TYPECPX *samples ;
     if( transfer->ctx == NULL ) {
         return(0) ;
     }

    struct t_rx_device* my_device = (struct t_rx_device*)transfer->ctx ;
    if( my_device->acq_stop == true ) {
        if( DEBUG_DRIVER ) fprintf(stderr,"%s(len=%d) my_device->acq_stop == true\n", __func__, transfer->sample_count );
        if( DEBUG_DRIVER ) fflush(stderr);
        return(0) ;
    }

    int sample_count = transfer->sample_count ;
    samples = (TYPECPX *)malloc( sample_count * sizeof( TYPECPX ));
    if( samples == NULL ) {
        if( DEBUG_DRIVER ) fprintf(stderr,"%s(len=%d) samples == NULL\n", __func__, sample_count );
        if( DEBUG_DRIVER ) fflush(stderr);
        return(0) ;
    }

    memcpy( (void *)samples, transfer->samples,  transfer->sample_count * sizeof(TYPECPX) ) ;

    // push samples to SDRNode callback function
    // we only manage one channel per device
    if( (*acqCbFunction)( my_device->uuid, (float *)samples, sample_count, 1, &my_device->ext_context ) <= 0 ) {
        free(samples);
    }
    return(0) ;
}

/**
 * @brief acquisition_thread This function is locked by the mutex and waits before starting the acquisition in asynch mode
 * @param params
 * @return
 */
void* acquisition_thread( void *params ) {
    int rc ;
    struct t_rx_device* my_device = (struct t_rx_device*)params ;    
    airspy_device *device = my_device->device ;
    if( DEBUG_DRIVER ) fprintf(stderr,"%s() start thread\n", __func__ );
    for( ; ; ) {
        my_device->running = false ;
        if( DEBUG_DRIVER ) fprintf(stderr,"%s() thread waiting\n", __func__ );
        if( DEBUG_DRIVER ) fflush(stderr);
        sem_wait( &my_device->mutex );
        if( DEBUG_DRIVER ) fprintf(stderr,"%s() airspy_start_rx\n", __func__ );

        rc = airspy_start_rx(device, sdr_callback, (void *)my_device );
        if( rc == AIRSPY_SUCCESS ) {
            my_device->running = true ;
            while (airspy_is_streaming(device) == AIRSPY_TRUE) {
                 my_device->running = true ;
                 usleep(10000);
            }
        }

    }
    return(NULL);
}


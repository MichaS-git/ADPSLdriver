 /* PSLdriver.cpp
 *
 * This is a driver for the Photonic Science X-Ray FDS detectors
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
using namespace std;

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsExit.h>
#include <iocsh.h>

#include "ADDriver.h"

#include <epicsExport.h>
#include <fdscontrol.h>

static const char *driverName = "PSLdriver";


/**
 * Driver for PSL FDS cameras using their fdscontrol.dll; inherits from ADDriver class in ADCore.
 */
class PSLdriver : public ADDriver {
public:
    PSLdriver(const char *portName, char *filesPath, int maxBuffers, size_t maxMemory,
              int priority, int stackSize);

    /* override ADDriver methods */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    //virtual void report(FILE *fp, int details);

    /* "private methods" that need to be called from C */
    void shutdown();
    void imageTask();
    void tempTask();

protected:
    int PSLPixelSpeed;
    #define FIRST_PSL_PARAM PSLPixelSpeed
    int PSLPostProcessing;
    int PSLOffsetSubtraction;
    int PSLBrightPixelCorrection;
    int PSLFlatFieldCorrection;
    int PSLBrightCornerSubtraction;
    int PSLSpotReduction;
    int PSLSharpening;
    #define LAST_PSL_PARAM PSLSharpening
private:
    asynStatus setGeometry();
    asynStatus getGeometry();
    asynStatus readParameters();
    int framesRemaining;
    int exiting_;
    epicsEventId startEvent_;
};
#define NUM_PSLdriver_PARAMS ((int)(&LAST_PSL_PARAM - &FIRST_PSL_PARAM + 1))

/* PSL driver specific parameters */
#define PSLPixelSpeedString                 "PSL_PIXEL_RATE"                /* (asynInt32,    r/w) 0 = 12.5 MHz  1 = 25 MHz */
#define PSLPostProcessingString             "PSL_POST_PROCESSING"           /* (asynInt32,    r/w) enable/disable post snap processing */
#define PSLOffsetSubtractionString          "PSL_OFFSET_SUBTRACTION"        /* (asynInt32,    r/w) enable/disable offset correction */
#define PSLBrightPixelCorrectionString      "PSL_BRIGHT_PIXEL_CORRECTION"   /* (asynInt32,    r/w) enable/disable bright pixel correction */
#define PSLFlatFieldCorrectionString        "PSL_FLAT_FIELD_CORRECTION"     /* (asynInt32,    r/w) enable/disable flat correction */
#define PSLBrightCornerSubtractionString    "PSL_BRIGHT_CORNER_SUBTRACTION" /* (asynInt32,    r/w) enable/disable bright corner correction */
#define PSLSpotReductionString              "PSL_SPOT_REDUCTION"            /* (asynInt32,    r/w) enable/disable intensifier spot reduction */
#define PSLSharpeningString                 "PSL_SHARPENING"                /* (asynInt32,    r/w) enable/disable sharpening */

static void c_shutdown(void *arg)
{
  PSLdriver *p = (PSLdriver *)arg;
  p->shutdown();
}

static void c_temptask(void *arg)
{
  PSLdriver *p = (PSLdriver *)arg;
  p->tempTask();
}

static void c_imagetask(void *arg)
{
  PSLdriver *p = (PSLdriver *)arg;
  p->imageTask();
}

void PSLdriver::tempTask(void)
{
    int status = asynSuccess;
    double dVal;
    static const char *functionName = "tempTask";

    while(!exiting_) {

        dVal = PSL_VHR_read_PLD_temperature();
        status |= setDoubleParam(ADTemperatureActual, dVal);
        if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: error, status=%d\n", driverName,
                              functionName, status);
        epicsThreadSleep(1.0);
        }
}

void PSLdriver::imageTask()
{
    NDArrayInfo   arrayInfo;
    epicsTimeStamp imageStamp;
    int acquire, pProcessing;
    unsigned short *image;
    int count, number;
    int callback;
    int status = asynSuccess;
    static const char *functionName = "imageTask";

    lock();

    while(!exiting_) {
        getIntegerParam(ADAcquire, &acquire);
        if(!acquire) {
            //cout << acquire << " acquire\n";
            unlock();
            epicsEventWait(startEvent_);
            lock();
        }
        status = !PSL_VHR_Snap_and_return();
        if(status) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: PSL_VHR_Snap_and_return error: %d\n",
                      driverName, functionName, status);
        }
        unlock();

        status = PSL_VHR_Get_snap_status();
        while (!status) {
            //cout << "Inside waiting loop\n";
            status = PSL_VHR_Get_snap_status();
        }
        setIntegerParam(ADStatus, ADStatusReadout);
        lock();

        image = PSL_VHR_get_image_pointer();
        getIntegerParam(PSLPostProcessing, &pProcessing);
        if (pProcessing) {
            status = !PSL_VHR_apply_post_snap_processing(image);
            if(status) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: PSL_VHR_apply_post_snap_processing error: %d\n",
                          driverName, functionName, status);
            }
        }
        epicsTimeGetCurrent(&imageStamp);
        getIntegerParam(ADNumImagesCounter, &number);
        number++;
        setIntegerParam(ADNumImagesCounter, number);
        getIntegerParam(NDArrayCounter, &count);
        count++;
        setIntegerParam(NDArrayCounter, count);
        callParamCallbacks();

        getIntegerParam(NDArrayCallbacks, &callback);

        if(callback) {
            NDArray *pImage;
            size_t dims[2];
            int itemp;

            getIntegerParam(NDArraySizeX, &itemp); dims[0] = itemp;
            getIntegerParam(NDArraySizeY, &itemp); dims[1] = itemp;

            pImage = pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);

            if(pImage) {
                pImage->uniqueId = count;
                pImage->timeStamp = imageStamp.secPastEpoch + (imageStamp.nsec / 1.0e9);
                updateTimeStamp(&pImage->epicsTS);

                pImage->getInfo(&arrayInfo);
                setIntegerParam(NDArraySize,  (int)arrayInfo.totalBytes);
                memcpy(pImage->pData, image, arrayInfo.totalBytes);

                getAttributes(pImage->pAttributeList);
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                pImage->release();

            }
        }
        /* See if acquisition is done */
        if (this->framesRemaining > 0) this->framesRemaining--;
        //cout << framesRemaining << " framesRemaining\n";
        if (this->framesRemaining == 0) {
            //cout << "done!\n";
            setShutter(0);
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();
        }
    }
}

void PSLdriver::shutdown(void)
{
    int pslStatus;

    exiting_ = 1;
    /* close system and release resources */
    pslStatus = PSL_VHR_Close();
    if (pslStatus) {
        cout << "Disconnect successful!\n";
    }
    else {
        cout << "Disconnect faulty!\n";
    }
    epicsThreadSleep(1);
}

asynStatus PSLdriver::readParameters()
{
    int status = asynSuccess;
    int intVal;
    static const char *functionName = "readParameters";

    intVal = PSL_VHR_get_speed();
    status |= setIntegerParam(PSLPixelSpeed, intVal);

    status |= getGeometry();

    /* Call the callbacks to update the values in higher layers */
    callParamCallbacks();

    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: error, status=%d\n",
                      driverName, functionName, status);
    return((asynStatus)status);
}

asynStatus PSLdriver::setGeometry()
{
    int status = asynSuccess;
    int binX, binY, minY, minX, sizeX, sizeY, maxSizeX, maxSizeY;
    static const char *functionName = "setGeometry";

    /* Get all of the current geometry parameters from the parameter library */
    status = getIntegerParam(ADBinX, &binX);
    if (binX < 1) binX = 1;
    status = getIntegerParam(ADBinY, &binY);
    if (binY < 1) binY = 1;
    status = getIntegerParam(ADMinX, &minX);
    status = getIntegerParam(ADMinY, &minY);
    status = getIntegerParam(ADSizeX, &sizeX);
    status = getIntegerParam(ADSizeY, &sizeY);
    status = getIntegerParam(ADMaxSizeX, &maxSizeX);
    status = getIntegerParam(ADMaxSizeY, &maxSizeY);

    if (minX + sizeX > maxSizeX) {
        sizeX = maxSizeX - minX;
        setIntegerParam(ADSizeX, sizeX);
    }

    if (minY + sizeY > maxSizeY) {
        sizeY = maxSizeY - minY;
        setIntegerParam(ADSizeY, sizeY);
    }

    if(!status){
        status = !PSL_VHR_set_sub_area_coordinates(minX, sizeX, minY, sizeY, binX, binY);
    }

    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: error, status=%d\n",
                      driverName, functionName, status);
    return((asynStatus)status);
}

asynStatus PSLdriver::getGeometry()
{
    int status = asynSuccess;
    long minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
    long signed imageWidth, imageHeight;
    static const char *functionName = "getGeometry";

    maxSizeX = PSL_VHR_get_maximum_width();
    maxSizeY = PSL_VHR_get_maximum_height();

    status = setIntegerParam(ADMaxSizeX,  maxSizeX);
    status = setIntegerParam(ADMaxSizeY, maxSizeY);

    PSL_VHR_get_actual_sub_area_coordinates(&minX, &sizeX, &minY, &sizeY);

    status = setIntegerParam(ADMinX,  minX);
    status = setIntegerParam(ADMinY,  minY);
    status = setIntegerParam(ADSizeX, sizeX);
    status = setIntegerParam(ADSizeY, sizeY);

    imageWidth = PSL_VHR_get_width();
    imageHeight = PSL_VHR_get_height();

    status = setIntegerParam(NDArraySizeX, imageWidth);
    status = setIntegerParam(NDArraySizeY, imageHeight);

    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: error, status=%d\n",
                      driverName, functionName, status);
    return((asynStatus)status);
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus PSLdriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    static const char *functionName = "writeInt32";

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
        if(value) {
            int imageMode, numImages;
            getIntegerParam(ADImageMode, &imageMode);
            getIntegerParam(ADNumImages, &numImages);
            switch(imageMode) {
            case ADImageSingle:
                this->framesRemaining = 1;
                break;
            case ADImageMultiple:
                this->framesRemaining = numImages;
                break;
            case ADImageContinuous:
                this->framesRemaining = -1;
                break;
            }
            setIntegerParam(ADStatus, ADStatusAcquire);
            setShutter(1);
            epicsEventSignal(startEvent_);
        } else {
            this->framesRemaining = 0;
            //PSL_VHR_abort_snap();
        }
    } else if ((function == ADBinX) ||
        (function == ADBinY) ||
        (function == ADMinX) ||
        (function == ADSizeX) ||
        (function == ADMinY) ||
        (function == ADSizeY)) {
        setGeometry();
    } else if (function == ADTriggerMode) {
        PSL_VHR_set_trigger_mode(value);
    } else if (function == PSLPixelSpeed) {
        PSL_VHR_set_speed(value);
    } else if (function == PSLOffsetSubtraction) {
        PSL_VHR_enable_offset_subtraction(value ? true:false);
    } else if (function == PSLBrightPixelCorrection) {
        PSL_VHR_enable_bright_pixel_correction(value ? true:false);
    } else if (function == PSLFlatFieldCorrection) {
        PSL_VHR_enable_flat_field_correction(value ? true:false);
    } else if (function == PSLBrightCornerSubtraction) {
        PSL_VHR_enable_bright_corner_subtraction(value ? true:false);
    } else if (function == PSLSpotReduction) {
        PSL_VHR_enable_spotred(value ? true:false);
    } else if (function == PSLSharpening) {
        PSL_VHR_enable_sharpening(value ? true:false);
    } else {
        /* If this is not a parameter we have handled call the base class */
        if (function < FIRST_PSL_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }

    status = readParameters();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%d\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
    return((asynStatus)status);
}

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus PSLdriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    const char *paramName;
    static const char *functionName = "writeFloat64";

    getParamName(function, &paramName);
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= setDoubleParam(function, value);

    if (function == ADAcquireTime) {
        /* Photonic Science FDS uses integer milliseconds */
        status = !PSL_VHR_set_exposure((unsigned long)(value * 1e3));
    }

    status |= readParameters();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, name=%s, value=%f\n",
              driverName, functionName, status, function, paramName, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%f\n",
              driverName, functionName, function, paramName, value);
    return((asynStatus)status);
}


extern "C" int PSLdriverConfig(const char *portName, char *filesPath, int maxBuffers,
                               size_t maxMemory, int priority, int stackSize)
{
    new PSLdriver(portName, filesPath, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

/** Constructor for PSLdriver driver; most parameters are simply passed to
  * ADDriver::ADDriver.
  *
  * After calling the base class constructor this method creates a thread to
  * collect the images from the detector and sets reasonable default values for
  * the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  *
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] filesPath Path to the specific camera files.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the
  *            NDArrayPool for this driver is allowed to allocate. Set this to
  *            -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for
  *            this driver is allowed to allocate. Set this to -1 to allow an
  *            unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread
  *            if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if
  *            ASYN_CANBLOCK is set in asynFlags.
  */
PSLdriver::PSLdriver(const char *portName, char *filesPath, int maxBuffers,
                     size_t maxMemory, int priority, int stackSize)
    : ADDriver(portName, 1, NUM_PSLdriver_PARAMS, maxBuffers, maxMemory,
               asynEnumMask, asynEnumMask,
               ASYN_CANBLOCK,  /* ASYN_CANBLOCK=1 ASYN_MULTIDEVICE=0 */
               1,              /* autoConnect=1 */
               priority, stackSize),
               exiting_(0), framesRemaining(0)
{
    static const char *functionName = "PSLdriver";
    int status = asynSuccess;

    /* create PSL specific parameters */
    createParam(PSLPixelSpeedString,                asynParamInt32,    &PSLPixelSpeed);
    createParam(PSLPostProcessingString,            asynParamInt32,    &PSLPostProcessing);
    createParam(PSLOffsetSubtractionString,         asynParamInt32,    &PSLOffsetSubtraction);
    createParam(PSLBrightPixelCorrectionString,     asynParamInt32,    &PSLBrightPixelCorrection);
    createParam(PSLFlatFieldCorrectionString,       asynParamInt32,    &PSLFlatFieldCorrection);
    createParam(PSLBrightCornerSubtractionString,   asynParamInt32,    &PSLBrightCornerSubtraction);
    createParam(PSLSpotReductionString,             asynParamInt32,    &PSLSpotReduction);
    createParam(PSLSharpeningString,                asynParamInt32,    &PSLSharpening);

    /* set read-only parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);

    /* open and initialize camera */
    /* example from PSL is PSL_VHR_select_IPORT_device("00-11-1c-00-79-e7","[169.254.88.104]") */
    /* the GEV Device Selection window appears anyway to select the camera, so we use the */
    /* function with empty strings; also we don't need a specific cameraId because only */
    /* one camera can be connected */
    PSL_VHR_select_IPORT_device("","");

    status = !PSL_VHR_Open(filesPath);

    if (status) {
        cout << "Camera not connected!\n";
    }

    status = PSL_VHR_Is_14bit_camera(); /* this is true for FDS */
    if (status) {
        cout << "This camera is non-color 14 bit (uses Cyclops pping but without the color processing section).\n";
    }

    status = PSL_VHR_Is_Cyclops_camera(); /* this is NOT true for FDS */
    if (status) {
        cout << "This camera is a Cyclops camera (and so is returning 16 bit data).\n";
    }

    status  = setStringParam(ADManufacturer, "Photonic Science");

    status |= readParameters();

    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: failed to register all features\n",
            driverName, functionName);
    }

    startEvent_ = epicsEventCreate(epicsEventEmpty);

    /* launch image read task */
    epicsThreadCreate("PSLdriverImageTask",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_imagetask, this);

    /* launch temp read task */
    epicsThreadCreate("PSLdriverTempTask",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_temptask, this);

    /* shutdown on exit */
    epicsAtExit(c_shutdown, this);
}


/* Code for iocsh registration */
static const iocshArg PSLdriverConfigArg0 = {"Port name", iocshArgString};
static const iocshArg PSLdriverConfigArg1 = {"cameraFilesPath", iocshArgString};
static const iocshArg PSLdriverConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg PSLdriverConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg PSLdriverConfigArg4 = {"priority", iocshArgInt};
static const iocshArg PSLdriverConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const PSLdriverConfigArgs[] =  {&PSLdriverConfigArg0,
                                                     &PSLdriverConfigArg1,
                                                     &PSLdriverConfigArg2,
                                                     &PSLdriverConfigArg3,
                                                     &PSLdriverConfigArg4,
                                                     &PSLdriverConfigArg5,};
static const iocshFuncDef configPSLdriver = {"PSLdriverConfig", 6, PSLdriverConfigArgs};
static void configPSLdriverCallFunc(const iocshArgBuf *args)
{
    PSLdriverConfig(args[0].sval, args[1].sval, args[2].ival,  args[3].ival,
                 args[4].ival, args[5].ival);
}

static void PSLdriverRegister(void)
{
    iocshRegister(&configPSLdriver, configPSLdriverCallFunc);
}

extern "C" {
epicsExportRegistrar(PSLdriverRegister);
}

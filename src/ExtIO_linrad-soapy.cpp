/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org>
*/


//#define Sleep(msec) usleep((msec)*1000)

#define FALSE false
#define TRUE true

// --------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
//#include <math.h>
#include <string.h>
#include "ExtIO_linrad-soapy.h"
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Version.hpp>

//using namespace std;

#define EXTIO_HWTYPE_16B 3

//#define DEBUG 0

//Globals, parameter the extio user sets
static long freq = 145000000;			// nominal / desired frequency
static long freqmin,freqmax;
static int gain,gainmin,gainmax;
static long samplerate;

//Other Globals
static int extio_blocksize;		// number of samples in one ExtIO block. unit: complex sample (= 2 real samples)
static int extio_bps=100;           // blocks per seconds targeted
volatile static bool doThreadExit = false;
static bool Running = FALSE;

// Soapy stuff
SoapySDR::Device *device=NULL;
size_t channel=0;
SoapySDR::Stream *stream=NULL;
size_t soapy_mtu;

struct ThreadContainer 
{
	long samplerate;
    int blocksize;  // ExtIO says it's int, so it's int. not size_t
    size_t soapy_mtu;
};

struct ThreadContainer ThreadVariables; // TODO better name if needed at all

// Thread handle
pthread_t *worker_handle=NULL;
pthread_t worker_handle_data;

void* ThreadProc(ThreadContainer*);
int Start_Thread();
int Stop_Thread();

/* ExtIO Callback */
void (* WinradCallBack)(int, int, float, void *) = NULL;
#define WINRAD_SRCHANGE 100
#define WINRAD_LOCHANGE 101
#define WINRAD_ATTCHANGE 125
#define HDSDR_RX_IQ		134
#define WINRAD_LOBLOCKED 102
#define WINRAD_LORELEASED 103
#define HDSDR_RX_LEFT	131

int Message(const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    int rv=vfprintf(stderr,format,ap);
    fputc('\n',stderr);
    va_end(ap);
    return rv;
}

void setsr(long sr) {
    device->setSampleRate(SOAPY_SDR_RX,channel,(double)sr);
    samplerate=(long)device->getSampleRate(SOAPY_SDR_RX,channel);
    Message("Actual sample rate is %d",samplerate);

    soapy_mtu=device->getStreamMTU(stream);
    if (soapy_mtu>=16384 && (soapy_mtu&511)==0) extio_blocksize=soapy_mtu;  // that lower limit should depend on samplerate and extio_bps, actually
    else {
        int min512=soapy_mtu>>9;
        if (soapy_mtu&511) min512++;

        extio_blocksize=((samplerate+(extio_bps>>1)) / extio_bps)>>9;
        //Message("target blocksize in 512samples units: %d. extio_bps=%d",extio_blocksize,extio_bps);
        if (extio_blocksize<min512) extio_blocksize=min512;
        extio_blocksize=extio_blocksize<<9;
    }
    Message("soapy MTU %d, blocksize %d",soapy_mtu,extio_blocksize);
}
extern "C"
bool  EXTIO_API InitHW(char *name, char *model, int& type)
{
    //  get device selector string from env
    //  make the device with soapy
    //  maybe also set up stream just to be sure channel 0 is available
    samplerate=8000000;
    Message("InitHW called");
    strcpy(name,"SoapySDR");
    type = EXTIO_HWTYPE_16B;
    const char *devspec=getenv("LINRAD_SOAPY_DEV");
    if (devspec==NULL) devspec="";
    device=SoapySDR::Device::make(devspec);
    if (device==NULL) {
        Message("SoapySDR::Device::make(\"%s\") failed",devspec);
        *model='\0';
        return FALSE;
    }
    auto drivername=device->getDriverKey();
    strcpy(model,drivername.c_str());
    channel=0;
    stream=device->setupStream(SOAPY_SDR_RX,"CS16",std::vector<size_t>(1,channel));
    if (stream==NULL) {
        Message("SoapySDR::Device::setupStream() failed");
        return FALSE;
    }
    setsr(samplerate);
    auto gr=device->getGainRange(SOAPY_SDR_RX,channel);
    gainmax=gr.maximum();
    gainmin=gr.minimum();
    auto frl=device->getFrequencyRange(SOAPY_SDR_RX,channel);
    freqmax=frl.back().maximum();
    freqmin=frl.front().minimum();
    Message("Freq range: %d-%d",freqmin,freqmax);

    Message("InitHW returns");
    return TRUE;
}

extern "C"
bool  EXTIO_API OpenHW()
{
    Message("OpenHW called");
    Message("OpenHW returns");
	return TRUE;
}

extern "C"
long EXTIO_API SetHWLO(long rfreq)
{
    Message("SetHWLO called: %d (min/max: %d/%d)",rfreq,freqmin,freqmax);
	//WinradCallBack(-1, WINRAD_LOBLOCKED, 0, NULL);
	if (rfreq > freqmax)
	{
		Message("Warning Out of Range");
		//WinradCallBack(-1, WINRAD_LORELEASED, 0, NULL);
		return freqmax;
	}
	if (rfreq < freqmin)
	{
		Message("Warning Out of Range");
		//WinradCallBack(-1, WINRAD_LORELEASED, 0, NULL);
		return -freqmin;
	}
    device->setFrequency(SOAPY_SDR_RX,channel,(double)rfreq);
    freq=device->getFrequency(SOAPY_SDR_RX,channel);
    Message("Actual freq: %d",freq);

    //WinradCallBack(-1, WINRAD_LOCHANGE, 0, NULL);
    Message("SetHWLO returns");
	return 0;
}

extern "C"
int EXTIO_API StartHW(long rfreq)
{
    Message("StartHW called");
    if (SetHWLO(rfreq)!=0) return -1;
    Start_Thread();
    Message("StartHW returns");
	return extio_blocksize;
}

extern "C" long EXTIO_API GetHWLO() {return freq;}

extern "C" long EXTIO_API GetHWSR() {return samplerate;}

/* meh, linrad does not actually have UI for setting sample rate for ExtIO device
 * also linrad defines this as SetHWSR(int), not SetHWSR(long)
extern "C"
long EXTIO_API SetHWSR(int WantedSR)
{
    SampleRateIdx = FindSampleRateIdx(WantedSR);
    WinradCallBack(-1, WINRAD_SRCHANGE, 0, NULL);
	return GetHWSR();
}
*/

extern "C"
void EXTIO_API SetRFGain(int rgain)
{
    Message("SetRFGain called");
    // linrad sets gain -6..44
    int mygain=rgain+6; // now 0..50
    if (mygain<0) mygain=0;
    if (mygain>(gainmax-gainmin)) mygain=gainmax-gainmin;
    device->setGain(SOAPY_SDR_RX,channel,gainmin+mygain);
    gain=mygain-6;
    Message("SetRFGain returns");
}


extern "C" int EXTIO_API GetRFGain(void) {return gain;}
/*
extern "C" int EXTIO_API ExtIoGetSrates(int srate_idx, double * samplerate) {return 1;}	// ERROR
extern "C" int EXTIO_API ExtIoGetActualSrateIdx(void) {return 0;}
extern "C" int EXTIO_API ExtIoSetSrate(int srate_idx) {return 1;}	// ERROR
extern "C" int EXTIO_API GetAttenuators(int atten_idx, float * attenuation) {return 1;}	// ERROR
extern "C" int EXTIO_API GetActualAttIdx(void) {return 0;}
extern "C" int EXTIO_API SetAttenuator(int atten_idx) {return 1;}	// ERROR
extern "C" int EXTIO_API ExtIoGetAGCs(int agc_idx, char * text) {return 1;}	// ERROR
extern "C" int EXTIO_API ExtIoGetActualAGCidx(void) { return 0;}
extern "C" int EXTIO_API ExtIoSetAGC(int agc_idx) {return 1;}	// ERROR
extern "C" int EXTIO_API ExtIoGetSetting(int idx, char * description, char * value) {return 1;}
extern "C" void EXTIO_API ExtIoSetSetting(int idx, const char * value) {return;}
*/

extern "C" void EXTIO_API StopHW() {
    Message("StopHW called");
	Running = FALSE;
	Stop_Thread();
    if (stream) {
        device->closeStream(stream);
        stream=NULL;
    }
    if (device) {
        SoapySDR::Device::unmake(device);
        device=NULL;
    }
    Message("StopHW returns");
}

extern "C" void EXTIO_API CloseHW() {
    Message("CloseHW called");
    Message("CloseHW returns");
}

/*
extern "C"
void EXTIO_API ShowGUI()
{
	//if (h_dialog == NULL)
	//	h_dialog = CreateDialog(hInst, MAKEINTRESOURCE(IDD_SDRPLAY_SETTINGS), NULL, (DLGPROC)MainDlgProc);
	//ShowWindow(h_dialog,SW_SHOW);
	//SetForegroundWindow(h_dialog);
}
*/

/*
extern "C"
void EXTIO_API HideGUI()
{
	//ShowWindow(h_dialog,SW_HIDE);
}
*/

/*
extern "C"
void EXTIO_API SwitchGUI()
{
	//if (IsWindowVisible(h_dialog))
	//	ShowWindow(h_dialog,SW_HIDE);
	//else
	//	ShowWindow(h_dialog,SW_SHOW);
}
*/

extern "C" void EXTIO_API SetCallback(void(*myCallBack)(int, int, float, void *)) {WinradCallBack = myCallBack;}

extern "C" int EXTIO_API GetStatus() {return 0;}

// end of extio interface

int Start_Thread()
{
    int rv;
	//If already running, exit
	if (worker_handle != NULL)
	{
		Message("Start Thread return -1");
		return -1;
	}

	doThreadExit = false;
	//Set Threadvariables, Isolates variables in thread from dialog box activity.
    // no more dialog box activity, but keept this just in case
	ThreadVariables.samplerate = samplerate;
	ThreadVariables.blocksize = extio_blocksize;
	ThreadVariables.soapy_mtu = soapy_mtu;
	//worker_handle = (HANDLE)_beginthread((void(*)(void*))ThreadProc, 0, (void*)&ThreadVariables);	
    rv=pthread_create(&worker_handle_data,NULL,(void*(*)(void*))ThreadProc,(void*)&ThreadVariables);
	//if (worker_handle == INVALID_HANDLE_VALUE)
	if (rv!=0) {
		Message("Start Thread return error: %s",strerror(rv));
		return -1;
	}
    worker_handle=&worker_handle_data;
    // TODO but this will need root
	//SetThreadPriority(worker_handle, THREAD_PRIORITY_TIME_CRITICAL);
	return 0;
}

int Stop_Thread()
{
	if(worker_handle == NULL)
		return -1;
	doThreadExit = true;
	//WaitForSingleObject(worker_handle,INFINITE);
    pthread_join(*worker_handle,NULL);
	worker_handle=NULL;
	// CloseHandle(worker_handle);		//  _endthread automatically closes the thread handle!
	return 0;
}

void* ThreadProc(ThreadContainer* myvars)
{
    //long samplerate=myvars->samplerate;
    int blocksize=myvars->blocksize;
    size_t mtu=myvars->soapy_mtu;
    int soapyflags;
    long long soapytime;
    Message("thread start");

	short *buffer = NULL;
    short *writeptr,*readptr;

	//buffer = (short *)calloc(extio_blocksize, 2*sizeof(short));
	buffer = (short *)malloc(3*blocksize*2*sizeof(short));
	if (buffer == NULL)
	{
		Message("buffer allocation failed");
		goto cleanUpThread;
	}

    // activate stream
    int irv;
    if ((irv=device->activateStream(stream))!=0) {
        Message("activateStream failed: %d",irv);
        goto cleanUpThread;
    }

    writeptr=readptr=buffer;
    // so - we have buffer, write to it in soapy_mtu chunks
    // read from it in extio_blocksize chunks
    Message("Thread mtu=%d, blocksize=%d",mtu,blocksize);
	while (!doThreadExit)
	{
        // repeat reading into buffer at writeptr until we have extio_blocksize sample at least
        while ((writeptr-readptr) < (blocksize*2)) {
            size_t need=blocksize*2 - (writeptr-readptr);
            if (need>mtu) need=mtu;
            int red=device->readStream(stream,(void**)&writeptr,need,soapyflags,soapytime);
            if (red<0) {
                Message("readStream failed: %d",red);
                goto cleanUpThread;
            }
            writeptr+=red*2;
        }
        // pass all available data
        while ((writeptr-readptr) >= (blocksize*2)) {
            WinradCallBack(blocksize, 0, 0, (void*)readptr);
            readptr+=blocksize*2;
        }
        if (readptr==writeptr) readptr=writeptr=buffer;
        else {
            // handle "leftover" data
            // move the data that lies between readptr .. writeptr to the beginning of teh buffer
            int nshorts=writeptr-readptr;
            //Message("This should not happen - copying %d shorts",nshorts);
            memcpy(buffer,readptr,nshorts*sizeof(*buffer));
            readptr=buffer;
            writeptr=buffer+nshorts;
        }
	}
    Message("thread stops, doThreadExit=%d",doThreadExit);
    device->deactivateStream(stream);

cleanUpThread:
	// avoid memory leaks!
	if (buffer) free(buffer);

	//_endthread();
    return NULL;
}


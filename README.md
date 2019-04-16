# linrad_extio_SoapySDR
extio glue allows to use SoapySDR with linrad. linux only. RX only.

only the absolute minimum is implemented

plus setting frequency and gain (just barely) in linrad.
SDR must support CS16 sample type.
samplerate is hardcoded at 8Msps in the source - go find it!
but you can set a device specifier in environment variable LINRAD_SOAPY_DEV - same as what SoapySDRUtil --probe and co takes.

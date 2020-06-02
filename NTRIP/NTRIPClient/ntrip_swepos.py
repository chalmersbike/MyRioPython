from NtripClient import NtripClient

# ntripclient = NtripClient(user="ChalmersE2RTK:885511",caster="192.71.190.141",port=80,mountpoint="MSM_GNSS",verbose=True)
ntripclient = NtripClient(user="ChalmersE2RTK:885511",caster="192.71.190.141",port=80,mountpoint="RTCM3_GNSS",verbose=True)
print ntripclient.readData()
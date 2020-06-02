#! /bin/bash
curl -v http://$3/$4# --header "Ntrip-Version: Ntrip/2.0" --header "User-Agent: NTRIP curl"

##usage user password server (inc port) mount
##curl --silent -f  --no-buffer --connect-timeout 10 -H "Ntrip-Version: Ntrip/2.0" -H "User-Agent: NTRIP CURL_NTRIP_TEST/0.1" -H "Authorization: Basic Q2hhbG1lcnNFMlJUSzo4ODU1MTE=" -u $1:$2  http://$3/$4
##curl -u $1:$2 http://$3/$4
#
##curl -v -H "GET MSM_GNSS HTTP/1.1" -H "User-Agent: NTRIP JCMBsoftPythonClient/0.2" -H "Authorization: Basic Q2hhbG1lcnNFMlJUSzo4ODU1MTE=" http://$3/$4
#curl -v -H "GET MSM_GNSS HTTP/1.1" -H "$GPGGA,145317.900,5741.2701,N,01158.7465,E,1,09,0.84,51.5,M,40.1,M,,*5F" -H "User-Agent: NTRIP JCMBsoftPythonClient/0.2" -H "Authorization: Basic Q2hhbG1lcnNFMlJUSzo4ODU1MTE=" -u $1:$2 http://$3/$4
#
##curl -H "GET $4 HTTP/1.1" -H "User-Agent: NTRIP JCMBsoftPythonClient/0.2" -H "Authorization: Basic Q2hhbG1lcnNFMlJUSzo4ODU1MTE=" -H "" -H "$GPGGA,145317.900,5741.2701,N,01158.7465,E,1,09,0.84,51.5,M,40.1,M,,*5F" http://$3/$4
##curl -H $'GET $4 HTTP/1.1\r\nUser-Agent: NTRIP JCMBsoftPythonClient/0.2\r\nAuthorization: Basic Q2hhbG1lcnNFMlJUSzo4ODU1MTE=\r\n\r\n$GPGGA,145317.900,5741.2701,N,01158.7465,E,1,09,0.84,51.5,M,40.1,M,,*5F\r\n' http://$3/$4
##curl -v -H $'GET MSM_GNSS HTTP/1.1\r\nUser-Agent: NTRIP JCMBsoftPythonClient/0.2\r\nAuthorization: Basic Q2hhbG1lcnNFMlJUSzo4ODU1MTE=\r\n\r\n$GPGGA,145317.900,5741.2701,N,01158.7465,E,1,09,0.84,51.5,M,40.1,M,,*5F\r\n' http://$3/$4
##curl -v -H $'GET MMSM_GNS HTTP/1.1\r\nUser-Agent: NTRIP JCMBsoftPythonClient/0.2\r\nAccept: */*\r\nConnection: close\r\n' http://$3/$4
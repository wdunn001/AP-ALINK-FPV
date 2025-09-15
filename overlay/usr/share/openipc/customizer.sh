#!/bin/sh

cli -s .isp.exposure 11
cli -s .video0.fps 90
cli -s .video0.size 1920x1080
cli -s .video0.bitrate 12000
cli -s .video0.codec h265
cli -s .video0.rcMode cbr
cli -s .video0.qpDelta -12
cli -s .video0.gopSize 1
cli -s .video1.enabled false
cli -s .video1.bitrate 8000
cli -s .video1.fps 60
cli -s .video1.codec h265
cli -s .video1.size 1920x1080
cli -s .outgoing.enabled true
cli -s .outgoing.server udp://224.0.0.1:5600
cli -s .outgoing.naluSize 1200
cli -s .records.split 1200
cli -s .records.notime true
cli -s .records.enabled false
cli -s .records.substream true
cli -s .records.maxUsage 99
cli -s .fpv.enabled true
cli -s .fpv.roiRect 0x64x160x640,160x64x192x640,928x64x192x640,1120x64x160x640
cli -s .fpv.noiseLevel 0




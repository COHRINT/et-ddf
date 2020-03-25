#!/bin/bash
ffmpeg -start_number 0 -i plt_%d.png -vcodec mpeg4 test.mp4

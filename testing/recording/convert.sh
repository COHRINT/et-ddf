#!/bin/bash
ffmpeg -start_number 0 -i plt_%d.png -vf fps=20 -vcodec mpeg4 test.mp4

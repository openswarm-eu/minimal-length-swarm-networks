#!/bin/bash
ffmpeg -framerate 60 -i frame_%10d.png -c:v libx264 -crf 23 "output.mp4"
# This python file takes a yml file containing 3 body position data
# and converts it into a WAV file with 3 tones, frequency modulated based on
# distance between bodies 12, 23 and 13.
# The output volume is scaled such that the maximum volume of the combined tones
# is set to maximum scale.
#
# Future options
# Stereo based on how far left/right the objects are.
# 3D positional audio adding up/down to the objects.
# Volume based on velocity or perhaps combined mass of objects.

import numpy as np
import matplotlib.pyplot as plt
from scipy.io.wavfile import read,write
from IPython.display import Audio
from numpy.fft import fft,ifft
import yaml
import math

FPS = 30.0              # Default frames per second
MAX_WAV_VALUE = 32767   # Using 16 bit signed 
VOLUME = .15            # Keep volume at a reasonable range
SAMPLING_RATE = 48000   # Default sampling rate
MINIMUM_FREQ = 50       # In Hz
MAXIMUM_FREQ = 2000     # Chopped a little for hard of hearing
SAMPLES_PER_FRAME = SAMPLING_RATE/FPS # How many samples are in 1 frame
SAMPLES_IN_TABLE =int(SAMPLING_RATE/MINIMUM_FREQ) # How many samples are in sine table
current_freq12 = 100    # Current frequency between 1 and 2
current_freq23 = 100    # Current frequency between 2 and 3
current_freq13 = 100    # Current frequency between 1 and 3
current_step12 = 1.0    # Steps in frequency curve. 100 hz = 1, 10,000 hz = 100
current_step23 = 1.0    # Steps in frequency curve. 100 hz = 1, 10,000 hz = 100
current_step13 = 1.0    # Steps in frequency curve. 100 hz = 1, 10,000 hz = 100
current_sample12 = 0.0  # Current sample number in sinewave curve
current_sample23 = 0.0  # Current sample number in sinewave curve
current_sample13 = 0.0  # Current sample number in sinewave curve
sineCurve=[]            # Empty list
max_dist = 0            # furthest distance between particles. Sets minimum frequency distance.
max_negx = -1           # Furthest left a particle reaches
max_posx = 1            # Furthest right a particle reaches
distances=[]            # Empty list
sampleOut=[]            # Output samples. 
wavOut=[]               # Output wave file data

# The index doesn't represent time in playback, but time in simulation.
# So when collisioncam is operating, the index is smaller, but the audio time is
# still the same.
if __name__ == '__main__':
  posData = yaml.load( open("threeBody.yml"), Loader = yaml.Loader)
  
  # Generate a sine table we can use for "frequency modulation"
  for i in range(SAMPLES_IN_TABLE):
    sineCurve.append(VOLUME*MAX_WAV_VALUE*math.sin(math.pi*2.0*i/SAMPLES_IN_TABLE))

  for posIndex in posData["PositionData"]:
    x1,y1,x2,y2,x3,y3 = posData["PositionData"][posIndex]
    x1 = float(x1)
    y1 = float(y1)
    x2 = float(x2)
    y2 = float(y2)
    x3 = float(x3)
    y3 = float(y3)
    dist12 = math.sqrt( (x1-x2)**2 + (y1-y2)**2 ) # order doesn't matter because of ^2
    dist23 = math.sqrt( (x3-x2)**2 + (y3-y2)**2 )
    dist13 = math.sqrt( (x1-x3)**2 + (y1-y3)**2 )
    if (x1 < max_negx):
      max_negx = x1
    if (x2 < max_negx):
      max_negx = x2
    if (x3 < max_negx):
      max_negx = x3

    if (x1 > max_posx):
      max_posx = x1
    if (x2 > max_posx):
      max_posx = x2
    if (x3 > max_posx):
      max_posx = x3
      
    if( dist12 > max_dist ):
      max_dist = dist12
    if( dist23 > max_dist ):
      max_dist = dist23
    if( dist13 > max_dist ):
      max_dist = dist13
    distances.append([dist12, x1, dist23, x2, dist13, x3])
  
  for(dist12,x1,dist23,x2,dist13,x3) in distances:
    current_freq12 = (1 - dist12/max_dist)*(MAXIMUM_FREQ - MINIMUM_FREQ) + MINIMUM_FREQ
    current_freq23 = (1 - dist23/max_dist)*(MAXIMUM_FREQ - MINIMUM_FREQ) + MINIMUM_FREQ
    current_freq13 = (1 - dist13/max_dist)*(MAXIMUM_FREQ - MINIMUM_FREQ) + MINIMUM_FREQ
    
    #Convert frequency to steps in the sinewave table, use float for precision
    current_step12 = 480/(SAMPLING_RATE * 1.0 / current_freq12)
    current_step23 = 480/(SAMPLING_RATE * 1.0 / current_freq23)
    current_step13 = 480/(SAMPLING_RATE * 1.0 / current_freq13)
    for i in range(int(SAMPLES_PER_FRAME)):
      sampleOut.append([sineCurve[int(current_sample12) % SAMPLES_IN_TABLE], x1,
                        sineCurve[int(current_sample23) % SAMPLES_IN_TABLE], x2,
                        sineCurve[int(current_sample13) % SAMPLES_IN_TABLE], x3])
      current_sample12 += current_step12 
      current_sample23 += current_step23
      current_sample13 += current_step13
  wavOut = np.ndarray((len(sampleOut), 2), dtype=np.int16)
  i = 0
  stereo_range = max_posx-max_negx
  #left left is (max_neg, max_neg+stereo_range/5)
  #left left has all sound energy in the left channel
  stereo_leftleft = (stereo_range/5) + max_negx 
  #left is (max_neg+stereo_range/5, max_neg + 2/5*stereo_range)
  #left has gradually more energy in the left channel
  stereo_left = stereo_leftleft + (stereo_range/5)
  #middle is (max_neg + 2/5*stereo_range, max_neg + 3/5*stereo_range)
  #middle has equal energy in both channels
  stereo_middle = stereo_left + (stereo_range/5)
  
  for out in sampleOut:
    
    left12=1.0
    left23=1.0
    left13=1.0
    right12=1.0
    right23=1.0
    right13=1.0
    
    wavOut[i] = (int((out[0]*left12+out[2]*left23+out[4]*left13)/3),
                 int((out[0]*right12+out[2]*right23+out[4]*right13)/3) ) # Just kludging the first two into left/right for now
    i = i + 1
  write("output.wav", SAMPLING_RATE, wavOut)
#!/usr/bin/env python

"""Calculate a running covariance

Bill Mania
bill@manialabs.us

Calculate a covariance based on a collection of samples. The
collection can be initialized and then grown to a specified
maximum size. Once the sample set reaches the maximum size,
it's treated like a ring buffer, adding the latest sample
to the newest end of the ring and removing the oldest
sample from the other end.

inputs:
    the existing sample list or None

    the new sample to add

    the number of samples in the existing sample

    the maximum number of samples

outputs:

    the covariance matrix

    the updated sample list

    the new number of samples
"""

import numpy

def calcCovariance(sampleList, newSample, numOfSamples, maxSamples):
    if numOfSamples == 0:
        sampleList = numpy.array([newSample])
    else:
        sampleList = numpy.append(sampleList, [newSample], 0)

    if numOfSamples == maxSamples:
        sampleList = numpy.delete(sampleList, 0, 0)
    else:
        numOfSamples = numOfSamples + 1
        
    covariance = numpy.cov(sampleList.T)

    return covariance, sampleList, numOfSamples


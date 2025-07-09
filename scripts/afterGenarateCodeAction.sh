#!/bin/bash

cd `dirname $0`
cd ../Core/Src

mv main.c main.cpp
mv stm32f7xx_it.c stm32f7xx_it.cpp

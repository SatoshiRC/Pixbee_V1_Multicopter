#!/bin/bash

cd `dirname $0`
cd ../Core/Src

mv main.cpp main.c
mv stm32f7xx_it.cpp stm32f7xx_it.c

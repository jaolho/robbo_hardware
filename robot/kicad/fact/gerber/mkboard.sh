#!/bin/sh

BOARD="roboplatform_fact"

rename .pho .gbr * 
rename ${BOARD}-B bottom * 
rename ${BOARD}-F top * 
rename ${BOARD}.drl drill_with_metallization.gbr * 
rename ${BOARD}-NPTH.drl drill_without_metallization.gbr * 
rename ${BOARD}-Edge_Cuts.gbr edge.gbr *

echo "Что же значат имена слоев платы $BOARD
bottom_ / top_ нижний или верхний слой
Cu -- медная фольга
Mask -- маска
SilkS -- Шелкография
Paste -- паяльная паста. Сейчас не используется

drill_ отверстия
with_metallization МЕТАЛЛИЗИРОВАННЫЕ переходные и отверстия для сквазного монтажа(DIP)
without_metallization НЕметаллизированные технические отверстия, например под крепежные болты

egde это края платы

" > readme.txt

zip ${BOARD}.zip *.gbr readme.txt

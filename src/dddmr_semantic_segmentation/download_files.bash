#!/bin/bash

mkdir ~/dddmr_bags

echo -n "Do you want to download rs435_rgbd_848x380 bag files (1.1GB)? (Y/N):"
read d_bag
if [ "$d_bag" != "${d_bag#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1Pqw93waRsJXrY6flw2somVmyQ4N4RJ0S \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o rs435_rgbd_848x380.zip \
      'https://drive.usercontent.google.com/download?id='1Pqw93waRsJXrY6flw2somVmyQ4N4RJ0S'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip rs435_rgbd_848x380.zip
fi


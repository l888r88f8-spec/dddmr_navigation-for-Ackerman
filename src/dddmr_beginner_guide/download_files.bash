#!/bin/bash

mkdir ~/dddmr_bags

echo -n "Do you want to download gazebo navigation demo map (12.3MB)? (Y/N):"
read d_bag
if [ "$d_bag" != "${d_bag#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1kCEyMnltRDkdUuS4Sr5CPOcsAguXX06F \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o gz_go2_demo_world.zip \
      'https://drive.usercontent.google.com/download?id='1kCEyMnltRDkdUuS4Sr5CPOcsAguXX06F'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip gz_go2_demo_world.zip
fi



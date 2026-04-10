#!/bin/bash

mkdir ~/dddmr_bags

echo -n "Do you want to download c16(leishen) bag files (4.2GB)? (Y/N):"
read d_bag
if [ "$d_bag" != "${d_bag#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='16oV9GdVns0CqJtM4bXcLrZ-41QxTQEb6 \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o weiwuyin_back_side.zip \
      'https://drive.usercontent.google.com/download?id='16oV9GdVns0CqJtM4bXcLrZ-41QxTQEb6'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip weiwuyin_back_side.zip
fi


echo -n "Do you want to download pose graph files (55MB)? (Y/N):"
read d_pg
if [ "$d_pg" != "${d_pg#[Yy]}" ] ;then 
  echo "Download pose graph"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='19Zo3gI5Fw-272M6I9iAP4e5XF1UvnANH \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o pose_graph_tutorial.zip \
      'https://drive.usercontent.google.com/download?id='19Zo3gI5Fw-272M6I9iAP4e5XF1UvnANH'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip pose_graph_tutorial.zip
fi

echo -n "Do you want to download airy(robosense) bag files (1.5GB)? (Y/N):"
read d_bag_airy
if [ "$d_bag_airy" != "${d_bag_airy#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1j0mblFEVl8gTwhGr0txQZ6zUGBJDTN-j \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o robosense_airy.zip \
      'https://drive.usercontent.google.com/download?id='1j0mblFEVl8gTwhGr0txQZ6zUGBJDTN-j'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip robosense_airy.zip
fi
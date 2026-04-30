#!/bin/bash

Help() {
  echo "Run without options to start 24-25WaterCode only"
  echo "Run with -H to start 24-25WaterCode + HoloOcean w/ Nvidia"
  echo "Run with -N to start 24-25WaterCode + HoloOcean w/o Nvidia"
}

while getopts "hHN" option; do
  case $option in
    h)
      Help
      exit;;
    H)
      docker compose --profile holoocean up -d --build
      exit;;
    N)
      docker compose --profile holoocean-no-nvidia up -d --build
      exit;;
    \?)
      Help
      exit;;
  esac
done

docker compose up -d --build

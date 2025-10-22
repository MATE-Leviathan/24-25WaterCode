#!/bin/bash

Help() {
  echo "Run without options to enter leviathan"
  echo "Run with -H to enter holoocean-ros"
}

while getopts "hH" option; do
  case $option in
    h)
      Help
      exit;;
    H)
      docker exec -it holoocean-ros /bin/bash
      exit;;
    \?)
      Help
      exit;;
  esac
done

docker exec -it leviathan /bin/bash

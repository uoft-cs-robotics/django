#!/usr/bin/env bash
if [ $# == 1 ]
then
  echo "Creating Directory <" $PWD"/"$1 ">"
  mkdir "$1"
  cd "$1"

  echo "Copying Dual Gen3 Moveit Config"
  cp -r $(catkin_find tor42_moveit)/. .
  echo "Updating Package"
  grep -rli 'tor42_moveit' * | xargs -i@ sed -i 's/tor42_moveit/'$1'/g' @
  echo "Done"

else
  echo "USAGE: customize_moveit.sh [new_moveit_package_name]"
fi

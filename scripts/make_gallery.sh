#!/bin/bash

function mergegifs() {
  local output_file="merged_output.gif"
  local fps=15
  local direction="h"  # Default is horizontal
  
  # Parse arguments
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -o|--output)
        output_file="$2"
        shift 2
        ;;
      --fps)
        fps="$2"
        shift 2
        ;;
      -v|--vertical)
        direction="v"
        shift
        ;;
      -h|--horizontal)
        direction="h"
        shift
        ;;
      *)
        echo "Unknown option: $1"
        echo "Usage: mergegifs [-o output_file] [--fps frame_rate] [-v|--vertical] [-h|--horizontal]"
        return 1
        ;;
    esac
  done
  
  # Check if any GIFs exist
  if ! ls *.gif &>/dev/null; then
    echo "Error: No GIF files found in current directory"
    return 1
  fi
  
  # List all GIF files in natural sort order
  local files=$(ls -v *.gif)
  
  # Build command components
  local inputs=""
  local filter=""
  local count=0
  
  for f in $files; do
    inputs="$inputs -i $f"
    if [ $count -gt 0 ]; then
      filter="$filter[$count:v]"
    else
      filter="[0:v]"
    fi
    count=$((count+1))
  done
  
  # Choose stack filter based on direction
  local stack_filter
  if [ "$direction" = "v" ]; then
    stack_filter="vstack"
    echo "Merging $count GIF files vertically..."
  else
    stack_filter="hstack"
    echo "Merging $count GIF files horizontally..."
  fi
  
  # Execute the ffmpeg command with fixed filter chain
  ffmpeg $inputs -filter_complex "$filter $stack_filter=inputs=$count:shortest=1[v];[v]fps=$fps,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -y "$output_file"
    
  if [ $? -eq 0 ]; then
    echo "Success! Created $output_file"
  else
    echo "Error: Failed to merge GIFs"
    return 1
  fi
}

mkdir -p tmp/gallery
rm -fr tmp/gallery

# Run and tumble
pogobatch -c conf/batch/gallery_simple.yaml -S ./examples/run_and_tumble/run_and_tumble -r 1 -t tmp/gallery/tmp -o tmp/gallery/results --keep-temp
cd tmp/gallery/tmp
a=0; for i in $(ls -d --indicator-style=none sim_instance_*); do gifski -r 20 -W 300 -H 300 --output run_and_tumble-$a.gif $i/frames/*png; let "a++"; done
mergegifs -o ../01-run_and_tumble.gif --fps 10
cd ..
rm -fr tmp
cd ../..

# Hanabi
pogobatch -c conf/batch/gallery_simple.yaml -S ./examples/hanabi/hanabi -r 1 -t tmp/gallery/tmp -o tmp/gallery/results --keep-temp
cd tmp/gallery/tmp
a=0; for i in $(ls -d --indicator-style=none sim_instance_*); do gifski -r 20 -W 300 -H 300 --output hanabi-$a.gif $i/frames/*png; let "a++"; done
mergegifs -o ../02-hanabi.gif --fps 10
cd ..
rm -fr tmp
cd ../..

# Phototaxis
pogobatch -c conf/batch/gallery_phototaxis.yaml -S ./examples/phototaxis/phototaxis -r 1 -t tmp/gallery/tmp -o tmp/gallery/results --keep-temp
cd tmp/gallery/tmp
a=0; for i in $(ls -d --indicator-style=none sim_instance_*); do gifski -r 20 -W 300 -H 300 --output phototaxis-$a.gif $i/frames/*png; let "a++"; done
mergegifs -o ../03-phototaxis.gif --fps 10
cd ..
rm -fr tmp
cd ../..

# Walls and membranes
pogobatch -c conf/batch/gallery_walls.yaml -S ./examples/walls/walls -r 1 -t tmp/gallery/tmp -o tmp/gallery/results --keep-temp
cd tmp/gallery/tmp
a=0; for i in $(ls -d --indicator-style=none sim_instance_*); do gifski -r 20 -W 300 -H 300 --output walls-$a.gif $i/frames/*png; let "a++"; done
mergegifs -o ../04-walls.gif --fps 10
cd ..
rm -fr tmp
cd ../..

# SSR
pogobatch -c conf/batch/gallery_ssr.yaml -S ./examples/ssr/ssr -r 2 -t tmp/gallery/tmp -o tmp/gallery/results --keep-temp
cd tmp/gallery/tmp
a=0; for i in $(ls -d --indicator-style=none sim_instance_*); do gifski -r 20 -W 300 -H 300 --output ssr-$a.gif $i/frames/*png; let "a++"; done
mergegifs -o ../05-ssr.gif --fps 10
cd ..
rm -fr tmp
cd ../..


# Merge
cd tmp/gallery
mergegifs -v -o ../../.description/gallery.gif --fps 10
cd ../..
#rm -fr tmp/gallery


# MODELINE "{{{1
# vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker

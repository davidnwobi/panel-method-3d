#!/usr/bin/env zsh

search_dir="${1:-.}"

cmake --build . --target=mainM -j=12

find "$search_dir" -type f -name "*.txt" | while IFS= read -r txt_file; do
	input_file="$txt_file"
	params_file="$(dirname "$txt_file")/params.dat"
	
	dir="$(dirname "$txt_file")"
	# Subfolder we'll store results in
	results_dir="$dir/results"
	
	if [[ -d "$results_dir" ]]; then
      rm -rf "$results_dir"
    fi
	
    # Create 'results' folder fresh
    mkdir -p "$results_dir"
	if [ -f "$params_file" ]; then
		./mainM -i "$input_file" -p "$params_file" -o "$results_dir" -r
	else
		echo "Warning: No params.dat found in $dir"
	fi
done

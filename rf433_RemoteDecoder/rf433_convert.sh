pulse=64
deviation=10
output=""
count=1;

calc() { awk "BEGIN{ printf \"%.2f\n\", $* }"; }

round() {
  printf "%.${2}f" "${1}"
}


for i in `cat values.txt`; do
	num_pulses=$(calc $i/$pulse)
	num_pulses_round=$(round ${num_pulses} 0)
	if [ $((count%2)) -eq 0 ]; then binary="0"; else binary="1";fi
	count=$((count+1))
	for i in $(seq 1 $num_pulses_round); do output="$output,$binary"; done
done
echo $output

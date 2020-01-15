#Choose functional type (1,2) to simulate
for cl_id in 1 2; do 
#Choose interstimulus interval
for interval_t in 0 10 20 30 40 50 60 70 80 90 100; do 
# Multiple simulations
for num_run in $(seq 0 49); do 	

	export 	LPARAMS="-c cl_id=${cl_id} -c interval_t=${interval_t} -c num_run=${num_run}"
	echo $LPARAMS
	../mod_files/x86_64/special -nogui $LPARAMS network_coincidence.hoc -
done
done 
done


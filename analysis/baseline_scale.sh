NUM_STEPS=5
DT=6000
THETA=0.5

# Not final testing script
# Doesn't involve frequency scaling disabling
# Core pinning
mkdir -p exp1 exp2 exp3 exp4 exp5 exp6 exp7 out


for i in $(seq 100000 100000 900000);
    do 
        
        ../vanilla ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp1/output_stat_$i.txt
        sleep 1
        ../postCOM ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp2/output_stat_$i.txt
        sleep 1
        ../postCOM_contig ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp3/output_stat_$i.txt
        sleep 1
        ../iterative ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp4/output_stat_$i.txt
        sleep 1
        ../dfsOrder ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp5/output_stat_$i.txt
        sleep 1
        ../bodyBlocking ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp6/output_stat_$i.txt
        sleep 1
        ../avx ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp7/output_stat_$i.txt
    
    done 

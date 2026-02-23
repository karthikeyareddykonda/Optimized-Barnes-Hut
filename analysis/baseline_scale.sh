NUM_STEPS=10
DT=6000
THETA=1

# Not final testing script
# Doesn't involve frequency scaling disabling
# Core pinning
mkdir -p exp1 exp2 exp3 out


for i in $(seq 100000 100000 900000);
    do 
        
        ../vanilla ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp1/output_stat_$i.txt
        sleep 1
        ../postCOM ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp2/output_stat_$i.txt
        sleep 1
        ../postCOM_contig ../input/input_$i.txt $NUM_STEPS $DT $THETA out/output_$i.txt > exp3/output_stat_$i.txt
    done 

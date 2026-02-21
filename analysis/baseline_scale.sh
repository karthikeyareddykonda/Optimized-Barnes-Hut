NUM_STEPS=10
DT=6000
THETA=1


for i in $(seq 100000 100000 900000);
    do 
        mkdir -p exp1
        ../vanilla ../input/input_$i.txt $NUM_STEPS $DT $THETA exp1/output_$i.txt > exp1/output_stat_$i.txt
    done 

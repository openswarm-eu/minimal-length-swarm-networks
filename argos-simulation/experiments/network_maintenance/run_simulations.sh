#!/bin/bash

# Get first parameter

EXPERIMENT_NAME=network_maintenance

LINE_SEED=11
LINE_FLOCK_DIST=73
LINE_SEPARATION=76
LINE_JOINING=77
LINE_TARGET_DISTANCE=85
LINE_EP_RAB_RANGE=116
LINE_RUN_NUMBER=108
LINE_TEAM_SIZE=123
WORKER_TYPE=worker

# Distance thresholds
FLOCK_DIST=15
SEPARATION_THRESHOLD=50
JOINING_THRESHOLD=45
TARGET_DISTANCE=60
EP_RAB_RANGE=(0.8 0.75 0.7 0.65 0.6 0.55 0.5)
DEFAULT_EP_RAB_RANGE=0.8

EXPERIMENT_DIR=<path-to-file>/minimal-length-swarm-networks/experiments/$EXPERIMENT_NAME
RESULT_DIR=<path-to-file>/minimal-length-swarm-networks/results/$EXPERIMENT_NAME
# RESULT_DIR=<path-to-file>/minimal-length-swarm-networks/results/temp
EXPERIMENT_FILE=$EXPERIMENT_DIR/${EXPERIMENT_NAME}.argos
SEED_FILE=$EXPERIMENT_DIR/seeds.txt

count=0
print_count=""

# Output color
ORANGE='\033[38;5;208m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Start timing
start=$(date +%s.%N)
date +%r


### RUN multiple experiments

for i in {3..6} # Number of teams
do
    
    # for j in 10 #12 18 24 # Number of workers per team
    # do
        
        for l in ${EP_RAB_RANGE[@]} # epuck rabn range
        do
            # Start timing
            start_config=$(date +%s.%N)

            seed_count=0

            # divide epuck rab range by DEFAULT_EP_RAB_RANGE
            factor=$(echo "scale=2; $l/$DEFAULT_EP_RAB_RANGE" | bc)

            # multiply EP_RAB_RANGE and DEFAULT_EP_RAB_RANGE and save to variable
            rab_range=$(echo "scale=2; $l" | bc)
            # echo "rab_range = $rab_range"
            flock_dist=$(echo "scale=2; $factor*$FLOCK_DIST" | bc)
            # echo "flock_dist = $flock_dist"
            sep_dist=$(echo "scale=2; $factor*$SEPARATION_THRESHOLD" | bc)
            # echo "sep_dist = $sep_dist"
            join_dist=$(echo "scale=2; $factor*$JOINING_THRESHOLD" | bc)
            # echo "join_dist = $join_dist"
            target_dist=$(echo "scale=2; $factor*$TARGET_DISTANCE" | bc)
            # echo "target_dist = $target_dist"

            # Set rab_range
            sed -i "${LINE_FLOCK_DIST}s/.*/<team_flocking target_distance='$flock_dist'/" $EXPERIMENT_FILE
            sed -i "${LINE_SEPARATION}s/.*/<team_distance separation_threshold='$sep_dist'/" $EXPERIMENT_FILE
            sed -i "${LINE_JOINING}s/.*/joining_threshold='$join_dist'\/>/" $EXPERIMENT_FILE
            sed -i "${LINE_TARGET_DISTANCE}s/.*/target_distance='$target_dist'\/>/" $EXPERIMENT_FILE
            sed -i "${LINE_EP_RAB_RANGE}s/.*/<epuck rab_range='$l'\/>/" $EXPERIMENT_FILE

            echo "rab_range = $rab_range, flock_dist = $flock_dist, sep_dist = $sep_dist, join_dist = $join_dist, target_dist = $target_dist"

            for k in {1..100} # Number of runs
            do
                # # go to next iteration
                # echo "i = $i, l = $l, k = $k"
                # continue

                let j=12

                # Start timing
                start_run=$(date +%s.%N)
                
                ((count++))
                ((seed_count++))

                # Set seed
                seed=`awk "NR==$seed_count {print; exit}" $SEED_FILE`
                sed -i "${LINE_SEED}s/.*/random_seed='$seed' \/>/" $EXPERIMENT_FILE

                # Set run number
                sed -i "${LINE_RUN_NUMBER}s/.*/run_number='$k'/" $EXPERIMENT_FILE

                # Set team size
                sed -i "${LINE_TEAM_SIZE}s/.*/<distribute_teams team_num='$i' max_per_team = '$j' robot_per_team='$j' center='0,0' density='0.2' worker_type='$WORKER_TYPE'\/>/" $EXPERIMENT_FILE

                # Set log directory number
                printf -v print_count "%03d" $k

                # Run experiment
                echo "Running experiment $count, No_teams = $i, robots_per_team = $j, rab_range=$l, run = $k, (s = $seed) ..."
                LOG_FILE="$RESULT_DIR/log.txt"
                LOG_ERR_FILE="$RESULT_DIR/log_err.txt"

                echo $LOG_FILE

                # Create folder if it doesn't exist
                if [ ! -d "${RESULT_DIR}" ]; then
                    mkdir -p "${RESULT_DIR}"
                    echo "Created folder: ${RESULT_DIR}"
                fi
                argos3 -c $EXPERIMENT_FILE -l $LOG_FILE -e $LOG_ERR_FILE
                # argos3 -c $EXPERIMENT_FILE
                NEW_LOG_FILE="$RESULT_DIR/${EXPERIMENT_NAME}_${i}T_${j}R_${l}RAB_$print_count/log.txt"
                NEW_LOG_ERR_FILE="$RESULT_DIR/${EXPERIMENT_NAME}_${i}T_${j}R_${l}RAB_$print_count/log_err.txt"
                # echo $LOG_FILE
                # echo $NEW_LOG_FILE
                mv $LOG_FILE $NEW_LOG_FILE
                mv $LOG_ERR_FILE $NEW_LOG_ERR_FILE
                # echo "done"

                file="$RESULT_DIR/${EXPERIMENT_NAME}_${i}T_${j}R_${l}RAB_$print_count/summary.csv"
                # Get the specified lines from the CSV file
                line1_num=9
                # line2_num=11
                line1=$(sed "${line1_num}q;d" ${file})
                # line2=$(sed "${line2_num}q;d" ${file})
                # Print the lines on the same line, with the first line in blue and the second line in green
                echo -e "\033[32m${line1}\033[0m"

                # End timing
                end=$(date +%s.%N)

                # Calculate the elapsed time in seconds
                elapsed_seconds_run=$(echo "$end - $start_run" | bc)

                # Convert the elapsed time to hours, minutes, and seconds
                hours_run=$(printf "%02d" $(echo "$elapsed_seconds_run / 3600" | bc) 2>/dev/null)
                minutes_run=$(printf "%02d" $(echo "($elapsed_seconds_run / 60) % 60" | bc) 2>/dev/null)
                seconds_run=$(printf "%02d" $(echo "$elapsed_seconds_run % 60" | bc) 2>/dev/null)

                # Print the elapsed time in hours, minutes, and seconds in orange color
                echo -e "${YELLOW}RUN took: ${hours_run}:${minutes_run}:${seconds_run}${NC}"

            done
        
            # End timing
            end=$(date +%s.%N)

            # Calculate the elapsed time in seconds
            elapsed_seconds_config=$(echo "$end - $start_config" | bc)
            elapsed_seconds=$(echo "$end - $start" | bc)

            # Convert the elapsed time to hours, minutes, and seconds
            hours_config=$(printf "%02d" $(echo "$elapsed_seconds_config / 3600" | bc) 2>/dev/null)
            minutes_config=$(printf "%02d" $(echo "($elapsed_seconds_config / 60) % 60" | bc) 2>/dev/null)
            seconds_config=$(printf "%02d" $(echo "$elapsed_seconds_config % 60" | bc) 2>/dev/null)
            hours=$(printf "%02d" $(echo "$elapsed_seconds / 3600" | bc) 2>/dev/null)
            minutes=$(printf "%02d" $(echo "($elapsed_seconds / 60) % 60" | bc) 2>/dev/null)
            seconds=$(printf "%02d" $(echo "$elapsed_seconds % 60" | bc) 2>/dev/null)

            echo -e "${YELLOW}CONFIG took: ${hours_config}:${minutes_config}:${seconds_config}${NC}"
            echo -e "${ORANGE}Elapsed time: ${hours}:${minutes}:${seconds}${NC}"
            date +%r
        done
    # done
done


### RUN one experiment

######

# # Parameters
# run_num=11
# print_count=""
# seed=619538
# team_num=3
# worker_per_team=12

# k="$run_num"
# i="$team_num"
# j="$worker_per_team"
# l=0
# m=12 # max per team

# l=0.5 # communication range

# # divide epuck rab range by DEFAULT_EP_RAB_RANGE
# factor=$(echo "scale=2; $l/$DEFAULT_EP_RAB_RANGE" | bc)

# # multiply EP_RAB_RANGE and DEFAULT_EP_RAB_RANGE and save to variable
# rab_range=$(echo "scale=2; $l" | bc)
# # echo "rab_range = $rab_range"
# flock_dist=$(echo "scale=2; $factor*$FLOCK_DIST" | bc)
# # echo "flock_dist = $flock_dist"
# sep_dist=$(echo "scale=2; $factor*$SEPARATION_THRESHOLD" | bc)
# # echo "sep_dist = $sep_dist"
# join_dist=$(echo "scale=2; $factor*$JOINING_THRESHOLD" | bc)
# # echo "join_dist = $join_dist"
# target_dist=$(echo "scale=2; $factor*$TARGET_DISTANCE" | bc)
# # echo "target_dist = $target_dist"

# # Set rab_range
# sed -i "${LINE_FLOCK_DIST}s/.*/<team_flocking target_distance='$flock_dist'/" $EXPERIMENT_FILE
# sed -i "${LINE_SEPARATION}s/.*/<team_distance separation_threshold='$sep_dist'/" $EXPERIMENT_FILE
# sed -i "${LINE_JOINING}s/.*/joining_threshold='$join_dist'\/>/" $EXPERIMENT_FILE
# sed -i "${LINE_TARGET_DISTANCE}s/.*/target_distance='$target_dist'\/>/" $EXPERIMENT_FILE
# sed -i "${LINE_EP_RAB_RANGE}s/.*/<epuck rab_range='$l'\/>/" $EXPERIMENT_FILE

# echo "rab_range = $rab_range, flock_dist = $flock_dist, sep_dist = $sep_dist, join_dist = $join_dist, target_dist = $target_dist"

# # Set seed
# sed -i "${LINE_SEED}s/.*/random_seed='$seed' \/>/" $EXPERIMENT_FILE

# # Set run number
# sed -i "${LINE_RUN_NUMBER}s/.*/run_number='$k'/" $EXPERIMENT_FILE

# # Set team size
# sed -i "${LINE_TEAM_SIZE}s/.*/<distribute_teams team_num='$i' max_per_team = '$m' robot_per_team='$j' center='0,0' density='0.2' worker_type='$WORKER_TYPE'\/>/" $EXPERIMENT_FILE

# # Set log directory number
# printf -v print_count "%03d" $k

# # Run experiment
# echo "Running experiment $count, No_teams = $i, robots_per_team = $j, rab_range=$l, run = $k, (s = $seed) ..."
# LOG_FILE="$RESULT_DIR/log.txt"
# LOG_ERR_FILE="$RESULT_DIR/log_err.txt"

# # echo $LOG_FILE

# # Create folder if it doesn't exist
# if [ ! -d "${RESULT_DIR}" ]; then
#     mkdir -p "${RESULT_DIR}"
#     echo "Created folder: ${RESULT_DIR}"
# fi
# argos3 -c $EXPERIMENT_FILE -l $LOG_FILE -e $LOG_ERR_FILE
# # argos3 -c $EXPERIMENT_FILE
# NEW_LOG_FILE="$RESULT_DIR/${EXPERIMENT_NAME}_${i}T_${j}R_${l}RAB_$print_count/log.txt"
# NEW_LOG_ERR_FILE="$RESULT_DIR/${EXPERIMENT_NAME}_${i}T_${j}R_${l}RAB_$print_count/log_err.txt"
# # echo $LOG_FILE
# # echo $NEW_LOG_FILE
# mv $LOG_FILE $NEW_LOG_FILE
# mv $LOG_ERR_FILE $NEW_LOG_ERR_FILE
# # echo "done"

# file="$RESULT_DIR/${EXPERIMENT_NAME}_${i}T_${j}R_${l}RAB_$print_count/summary.csv"
# # Get the specified lines from the CSV file
# line1_num=9
# # line2_num=11
# line1=$(sed "${line1_num}q;d" ${file})
# # line2=$(sed "${line2_num}q;d" ${file})
# # Print the lines on the same line, with the first line in blue and the second line in green
# echo -e "\033[32m${line1}\033[0m"

# ######

# End timing
end=$(date +%s.%N)

# Calculate the elapsed time in seconds
elapsed_seconds=$(echo "$end - $start" | bc)

# Convert the elapsed time to hours, minutes, and seconds
hours=$(printf "%02d" $(echo "$elapsed_seconds / 3600" | bc) 2>/dev/null)
minutes=$(printf "%02d" $(echo "($elapsed_seconds / 60) % 60" | bc) 2>/dev/null)
seconds=$(printf "%02d" $(echo "$elapsed_seconds % 60" | bc) 2>/dev/null)

# Print the elapsed time in hours, minutes, and seconds in orange color
ORANGE='\033[38;5;208m'
NC='\033[0m' # No Color
echo -e "${ORANGE}TOTAL elapsed time: ${hours}:${minutes}:${seconds}${NC}"

date +%r
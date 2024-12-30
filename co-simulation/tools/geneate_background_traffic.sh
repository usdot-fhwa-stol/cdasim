#!/bin/bash
#This script will generate random traffic to be used in Sumo-carla co simulation for a any Sumo Network File given the simulation duration, number of vehicles and a random number seed
# This script requires  installation of Sumo tools randomTrips.py and Python
echo "Sumo Network Files in this directoryi:"
ls *.net.xml
# Prompt for user inputs
echo 
echo "Enter the Sumo Network File Name:"
read net_file

echo "Output File Name:"
read out_file

echo "Enter Duration of the Simulation:"
read sim_duration

echo "How many vehicles you want to instantiate?:"
read veh_number

echo "Enter Random Number Seed (Different Seed will genrate different traffic):"
read seed

# Calculate the rate of vehicle generation
rate=$(echo "$sim_duration / $veh_number" | bc -l)


# Run the Python script with the provided inputs
python3 /usr/share/sumo/tools/randomTrips.py -n $net_file -r $out_file  --fringe-factor 5 -e $sim_duration -p $rate --vehicle-class passenger --validate --random --seed $seed


#Changing vehicle Types matching pasenger car of CARLA 
# List of Passenger vehicle classes in CARLA 

vehicle_classes=(
  "vehicle.audi.a2"
  "vehicle.audi.tt"
  "vehicle.jeep.wrangler_rubicon"
  "vehicle.chevrolet.impala"
  "vehicle.mini.cooper_s"
  "vehicle.mercedes.coupe"
  "vehicle.bmw.grandtourer"
  "vehicle.citroen.c3"
  "vehicle.ford.mustang"
  "vehicle.lincoln.mkz_2017"
  "vehicle.seat.leon"
  "vehicle.nissan.patrol"
  "vehicle.nissan.micra"
)

temp_file="temp.xml"

# Read the generated route file line by line
while IFS= read -r line; do
  if [[ "$line" == *'type="passenger"'* ]]; then
    # Select a random vehicle class
    random_vehicle=${vehicle_classes[$RANDOM % ${#vehicle_classes[@]}]}
    # Replace type="passenger" with the random vehicle class
    line=$(echo "$line" | sed "s/type=\"passenger\"/type=\"$random_vehicle\"/")
  fi
  # Write the updated line to a temporary file
  echo "$line" >> "$temp_file"
done < "$out_file"

# Replace the original file with the updated file
mv "$temp_file" "$out_file"
rm trips.trips.xml
echo "Route file $out_file generated"



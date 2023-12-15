Steps to run the code:

1. Clone the repository 

`sh
git clone https://github.com/viraj96/MAPF-PC-LNS.git
`

2. Switch to `fullPrecConstraints` branch for MAPF-PC and `kiva_tasks` for the MAPD approach
3. Clone the MAPF-PC submodule

`sh
cd MAPF-PC-LNS
git submodule update --init -recursive
`

4. Use the current MAPF-PC branch for MAPF-PC problems or `kiva_support` for MAPD problems
5. Run the following,

`sh
cd MAPF-PC
mkdir build
cd build
cmake ..
make
cd ..
mkdir build
cd build
cmake ..
make
cd ..
`

6. To run MAPF-PC problems run the following,

`sh
./build/mapf_pc_lns -m ./sample_input/empty-16-16.map -a ./sample_input/agent_goals.txt -k 10 -l 50 -t 10 -n 2 -i 200 -d 6 -s sota_pbs -h alns -c TA -r absolute
`

7. To run the MAPD problems run the following,

`sh
./build/mapf_pc_lns -m ./sample_input/kiva-3-500-5.map -a ./sample_input/0.task -k 3 -l 100 -t 10 -n 2 -i 200 -d 6 -s sota_pbs -h alsn -c TA -r absolute
`

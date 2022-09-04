all: clean
	mkdir build && cd build && cmake ../ && make

clean:
	rm -rf build

run_file:
	./build/cloud_viewer --pcd_path ../data/sim_data/00000000.pcd

run_dir:
	./build/cloud_viewer --pcd_path ../data/real_data

run_dummy:
	./build/cloud_viewer --pcd_path ../data_dumy
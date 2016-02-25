# Install PCL on ubuntu
echo "Installing PCL, please provide the root permission"
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all \
                     meshlab

# Builds the transfer_vertex_color binary
echo "Building transfer_vertex_color"
cd ./transfer_vertex_color
cmake .
make 

echo "Building texture_mapping"
cd ./texture_mapping
cmake .
make 

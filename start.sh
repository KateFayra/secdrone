echo "Starting Server..."
cd /home/pi/secdrone
echo "Killing python instances"
pkill -9 python
echo "USB reset"
sudo ./usbreset /dev/bus/usb/001/004
echo "Starting python server"
python server.py
echo "Killing python instances"
pkill -9 python

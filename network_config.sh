#!/bin/bash

ethernet_port="eth0"
wifi_adapter="wlan0"

if ! [ -f /etc/init/netconfig ]; then
    touch netconfig.sh
    echo '#!/bin/bash' >> netconfig.sh
    echo 'echo "Hello, World!"' >> netconfig.sh
    echo 'cd ~/Desktop' >> netconfig.sh
    echo 'touch "HelloWorld.txt"' >> netconfig.sh
    echo 'exit 0' >> netconfig.sh
    sudo chmod +x netconfig.sh

    sudo mv netconfig.sh /etc/init.d
fi












# #!/bin/bash

# if ! [ -f /etc/init.d/netconfig.sh ]; then
#     touch netconfig.sh
#     chmod +x netconfig.sh

#     echo "touch ~/Desktop/HelloWorld.txt" >> netconfig.sh

#     sudo mv netconfig.sh /etc/init.d
# fi
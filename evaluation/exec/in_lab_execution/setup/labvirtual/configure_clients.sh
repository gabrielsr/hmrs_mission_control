#!/bin/bash

#sudo cat ./hosts >> /etc/hosts

echo "Setting name servers...."

ssh-keygen -t rsa
declare -a arr=(1 2 3 4 5 6 7 8)
for i in "${arr[@]}"
do
    ssh-copy-id -i $HOME/.ssh/id_rsa.pub mission_control_exec@lab-0$i
done

# for i in "${arr[@]}"
# do
#     ssh lesunb@les-0$i
# done
echo "File automatically generated from git log." > AUTHORS
echo "To generated this file run ./utilities/docs/updateAuthors.sh from the root folder." >> AUTHORS
echo "" >> AUTHORS
echo "gazebo-yarp-plugins was written by these fine people:" >> AUTHORS 
echo "" >> AUTHORS
git log --format='%aN <%aE>' | sort -f | uniq >> AUTHORS

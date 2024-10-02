VERSION 0.7

FROM ros:humble

swerve-bot-humble-packages:

    COPY --dir ./swerve* /build-bed/
    WORKDIR /build-bed 
    
    RUN apt-get update
    RUN apt-get install -y python3-bloom python3-rosdep fakeroot debhelper dh-python
    RUN apt-get clean
    RUN rm -rf /var/lib/apt/lists/* 

    RUN rm /etc/ros/rosdep/sources.list.d/20-default.list 
    RUN sudo rosdep init
    RUN rosdep update
    RUN apt update
    RUN rosdep install --from-path . -y --skip-keys "swervebot_description"
    RUN echo "#!/bin/bash

# Find all directories containing package.xml
find . -type f -name "package.xml" | while read -r package_file; do
    dir="\$\(dirname "\$package_file"\)"
    echo "Processing directory: \$dir"

    # Change to the directory
    (cd "\$dir" || { echo "Failed to enter directory \$dir"; exit 1; }

    # Run the commands
    bloom-generate rosdebian
    fakeroot debian/rules binary
    )
done" > /binary-builder.sh

    RUN chmod +x /binary-builder.sh
    RUN /binary-builder.sh


    # SAVE ARTIFACT /build-bed/*.deb
    SAVE ARTIFACT /build-bed


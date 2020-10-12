function a = A(phi)

%A matrix transform (rotate) local coordinates of the body to the glodal coordinate system 

a = [cos(phi), -sin(phi); sin(phi), cos(phi)];

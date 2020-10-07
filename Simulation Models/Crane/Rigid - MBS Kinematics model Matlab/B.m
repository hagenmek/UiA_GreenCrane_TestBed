function a = B(phi)

%B matrix (A d/dt) rotate vector S 90deg and transform (rotate) local coordinates of the body to the glodal coordinate system 

a = [-sin(phi), -cos(phi); cos(phi), -sin(phi)];
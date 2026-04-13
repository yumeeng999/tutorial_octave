function matrix_R=rotate_y_axis(degree)

matrix_R=[cos(degree) 0 -sin(degree);
    0 1 0;
    sin(degree) 0 cos(degree);];

end
function y = SC(x,epsilon)

 % y = sin(x*pi + epsilon)/(x*pi + epsilon);
    % y = sinc(x*pi + epsilon)*(x*pi + epsilon);
    % y = sinc(x/pi + epsilon);
    y = sin(x + epsilon)/(x + epsilon);

end

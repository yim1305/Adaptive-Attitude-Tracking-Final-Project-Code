function [h, A] = cmg_model(delta, cmg)
    s = sin(delta);
    c = cos(delta);
    sb = sin(cmg.beta);
    cb = cos(cmg.beta);

    h = cmg.h0 * [-cb * s(1) - c(2) + cb * s(3) + c(4);
                    c(1) - cb * s(2) - c(3) + cb * s(4);
                    sb * sum(s)];

    A = [-cb * c(1),       s(2),  cb * c(3),      -s(4);
              -s(1), -cb * c(2),       s(3),  cb * c(4);
          sb * c(1),  sb * c(2),  sb * c(3),  sb * c(4)];
end

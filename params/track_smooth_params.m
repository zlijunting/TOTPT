function param = track_smooth_params(mode)

param = struct();

switch mode
    case 'smooth'
        param.lambda_e = 0.05;
        param.lambda_1 = 0;
        param.lambda_2 = 1;
        param.lambda_3 = 5;
        param.lambda_4 = 10;

    case 'mincurv'
        param.lambda_e = 0;
        param.lambda_1 = 0;
        param.lambda_2 = 1;
        param.lambda_3 = 5;
        param.lambda_4 = 40;

    otherwise
        error('Specify "smooth" or "mincurv" modes.')

end


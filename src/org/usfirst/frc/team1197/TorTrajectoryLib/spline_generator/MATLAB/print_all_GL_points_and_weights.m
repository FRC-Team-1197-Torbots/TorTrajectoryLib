function print_all_GL_points_and_weights(a,b,N)
%
% This function prints all Gauss-Legendre points and integration weights
% in [a,b] for 1 to N grid nodes in java-friendly formatting (w/o indent).
%
% Input a,b -> endpoints of the interval [a,b]
%         N -> maximum number of GL points in [a,b]

doubleformat = '% 17.15fD';
columns = 4;
clc;

%% points:
fprintf('public static final double points[][] = {\n');

for n = 1:N
    [z,w] = get_GL_points_and_weights(a,b,n);
    for i = 1:N
        if i == 1
            fprintf('{');
        else
            fprintf(',');
            if mod(i-1, columns) == 0
                fprintf('\n ');
            else
                fprintf(' ');
            end
        end
        if i <= n
            fprintf(doubleformat, z(i));
        else
            fprintf(doubleformat, 0.0);
        end
    end
    if n < N
        fprintf('},\n');
    else
        fprintf('}\n};\n');
    end
end

%% weights:
fprintf('\npublic static final double weights[][] = {\n');

for n = 1:N
    [z,w] = get_GL_points_and_weights(a,b,n);
    for i = 1:N
        if i == 1
            fprintf('{');
        else
            fprintf(',');
            if mod(i-1, columns) == 0
                fprintf('\n ');
            else
                fprintf(' ');
            end
        end
        if i <= n
            fprintf(doubleformat, w(i));
        else
            fprintf(doubleformat, 0.0);
        end
    end
    if n < N
        fprintf('},\n');
    else
        fprintf('}\n};\n');
    end
end

end


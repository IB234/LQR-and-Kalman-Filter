% clear and get rid of anything from prior runs.
clear;
clc;

% System 1
A1 = [1, 3; 2, 2];
B1 = [1; 1];
C1 = [1, -1];

% System 2
A2 = [0, 0.2, 0; 0, 0, 0.8; 0, 0, 0];
B2 = [0; 0; 1];
C2 = [1, 0, 0];


disp('--- System 1 ---');
analyze_system(A1, B1, C1);

disp(' ');  % empty line for better display


disp('--- System 2 ---');
analyze_system(A2, B2, C2);

disp(' ');  

% this is the declaration and implementation of the functions.


% This function checks all of the system's properties.
function analyze_system(A, B, C)
    check_stability(A);
    check_controllability(A, B);
    check_observability(A, C);
    check_detectability(A, C);
end

%% 1. Function to Check Stability
function is_stable = check_stability(A)
    eigenvals = eig(A);
    % Check if all eigenvalues are strictly inside the unit circle
    is_stable = all(abs(eigenvals) < 1);
    
    fprintf('Stability: %s\n', mat2str(is_stable));
    disp('  Eigenvalues:');
    disp(eigenvals.');
end

%% 2. Function to Check Controllability
function is_controllable = check_controllability(A, B)
    n = size(A, 1);
    C_mat = ctrb(A, B);
    rank_C = rank(C_mat);
    % Full row rank indicates controllability
    is_controllable = (rank_C == n);
    
    fprintf('Controllability: %s (Rank of C = %d)\n', mat2str(is_controllable), rank_C);
end

%% 3. Function to Check Observability
function is_observable = check_observability(A, C)
    n = size(A, 1);
    O_mat = obsv(A, C);
    rank_O = rank(O_mat);
    % Full column rank indicates observability
    is_observable = (rank_O == n);
    
    fprintf('Observability: %s (Rank of O = %d)\n', mat2str(is_observable), rank_O);
end

%% 4. Function to Check Detectability
function is_detectable = check_detectability(A, C)
    n = size(A, 1);
    eigenvals = eig(A);
    is_detectable = true; % Assume detectable until proven otherwise
    
    for i = 1:length(eigenvals)
        lambda = eigenvals(i);
        % Only check PBH rank for unstable modes
        if abs(lambda) >= 1 
            pbh_matrix = [A - lambda * eye(n); C];
            
            % If an unstable mode is hidden (rank < n), it is not detectable
            if rank(pbh_matrix) < n
                is_detectable = false;
                break; % Stop checking, one failure is enough
            end
        end
    end
    
    fprintf('Detectability: %s\n', mat2str(is_detectable));
end
function [R_est, error_angle] = two_vector_attitude(w1_G, w2_G, l1_B, l2_B, R_true)
    % 双矢量定姿算法
    % 输入：
    %   w1_G, w2_G - J2000系下的两个参考矢量（3×1，单位矢量）
    %   l1_B, l2_B - 本体系下的两个观测矢量（3×1，单位矢量）
    %   R_true - 真实旋转矩阵（可选，用于计算误差）
    % 输出：
    %   R_est - 估计的旋转矩阵（从J2000系到本体系）
    %   error_angle - 姿态估计误差（弧度）
    
    % 1. 构建参考系正交基（J2000系）
    wx_G = cross(w1_G, w2_G);
    wx_G = wx_G / norm(wx_G);
    
    wy_G = cross(w1_G, wx_G);
    wy_G = wy_G / norm(wy_G);
    
    wz_G = w1_G;  % 已为单位矢量
    
    M_ref = [wx_G, wy_G, wz_G];
    
    % 2. 构建观测系正交基（本体系）
    lx_B = cross(l1_B, l2_B);
    lx_B = lx_B / norm(lx_B);
    
    ly_B = cross( l1_B,lx_B);
    ly_B = ly_B / norm(ly_B);
    
    lz_B = l1_B;  % 已为单位矢量
    
    M_obs = [lx_B, ly_B, lz_B];
    
    % 3. 计算旋转矩阵
    R_est = M_obs * M_ref';
    
    % 4. 计算姿态误差（如果提供了真实矩阵）
    if nargin >= 5 && ~isempty(R_true)
        % 使用四元数方法计算误差角
        q_true = dcm2quat(R_true);
        q_est = dcm2quat(R_est);
        
        % 确保四元数在同一个半球
        if dot(q_true, q_est) < 0
            q_est = -q_est;
        end
        
        error_angle = 2 * acos(abs(dot(q_true, q_est)));
    else
        error_angle = 0;
    end
end
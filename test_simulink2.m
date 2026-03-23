% Check Task result
x_est = out.X_Est.signals.values(:, 1);
y_est = out.X_Est.signals.values(:, 2);
t = out.X_Est.time;

gt_pos = squeeze(out.GT_position.signals.values);
if size(gt_pos,1)==3, gt_pos=gt_pos'; end
gt_time = out.GT_position.time;
GT_x = interp1(gt_time, gt_pos(:,1), t, 'linear', 'extrap');
GT_y = interp1(gt_time, gt_pos(:,2), t, 'linear', 'extrap');

rmse = sqrt(mean((x_est - GT_x).^2 + (y_est - GT_y).^2));
fprintf('Task RMSE: %.4f m\n', rmse);

figure;
plot(GT_x-GT_x(1), GT_y-GT_y(1), 'k-', 'LineWidth', 2); hold on;
plot(x_est-GT_x(1), y_est-GT_y(1), 'b--', 'LineWidth', 1.5);
legend('Ground Truth','EKF'); grid on; axis equal;
title(sprintf('Task — RMSE=%.3fm', rmse));
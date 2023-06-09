function error_obs_weighted = error_obs_weighted_opt(X,Y,Z,d_x,d_y,d_z,fx,fy,rx,ry,rz,tx,ty,tz,u0,u_obs,v0,v_obs,vi,w_x,w_y,w_z)
%ERROR_OBS_WEIGHTED
%    ERROR_OBS_WEIGHTED = ERROR_OBS_WEIGHTED(X,Y,Z,D_X,D_Y,D_Z,FX,FY,RX,RY,RZ,TX,TY,TZ,U0,U_OBS,V0,V_OBS,VI,W_X,W_Y,W_Z)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    09-Mar-2022 09:43:00

t2 = d_x.*vi;
t3 = d_y.*vi;
t4 = d_z.*vi;
t5 = rx.^2;
t6 = ry.^2;
t7 = rz.^2;
t8 = t3+ty;
t9 = t4+tz;
t10 = t5+t6;
t11 = t5+t7;
t12 = t6+t7;
t13 = fy.*t8;
t14 = t9.*v0;
t15 = t7+t10;
t16 = 1.0./t15;
t17 = sqrt(t15);
t18 = 1.0./t17;
t19 = cos(t17);
t20 = sin(t17);
t21 = t19-1.0;
t22 = rx.*t18.*t20;
t23 = ry.*t18.*t20;
t24 = rz.*t18.*t20;
t25 = rx.*ry.*t16.*t21;
t26 = rx.*rz.*t16.*t21;
t27 = ry.*rz.*t16.*t21;
t31 = t10.*t16.*t21;
t32 = t11.*t16.*t21;
t33 = t12.*t16.*t21;
t28 = -t25;
t29 = -t26;
t30 = -t27;
t34 = t31+1.0;
t35 = t32+1.0;
t36 = t33+1.0;
t49 = t22+t27;
t50 = t23+t26;
t51 = t24+t25;
t37 = t34.*w_x;
t38 = t35.*w_x;
t39 = t36.*w_y;
t40 = t36.*w_z;
t42 = t34.*vi.*w_y;
t44 = t35.*vi.*w_z;
t52 = t49.*w_x;
t53 = t50.*w_x;
t54 = t51.*w_y;
t55 = t51.*w_z;
t56 = t22+t30;
t57 = t23+t29;
t58 = t24+t28;
t61 = t50.*vi.*w_y;
t63 = t49.*vi.*w_z;
t41 = t37.*vi;
t43 = t38.*vi;
t45 = t39.*vi;
t46 = t40.*vi;
t47 = -t37;
t48 = -t39;
t59 = t52.*vi;
t60 = t53.*vi;
t62 = t54.*vi;
t64 = t55.*vi;
t65 = t56.*w_x;
t66 = t58.*w_x;
t67 = t57.*w_y;
t68 = t57.*w_z;
t72 = t56.*vi.*w_y;
t74 = -t61;
t76 = t58.*vi.*w_z;
t84 = t38+t54;
t85 = t40+t53;
t98 = t42+t57+t63;
t69 = t65.*vi;
t70 = t66.*vi;
t71 = -t59;
t73 = t67.*vi;
t75 = t68.*vi;
t77 = -t64;
t80 = -t72;
t83 = -t76;
t86 = X.*t85;
t87 = Y.*t84;
t88 = t47+t68;
t89 = t48+t66;
t90 = -X.*(t39-t66);
t91 = -Z.*(t37-t68);
t92 = t52+t67;
t93 = t55+t65;
t99 = t43+t56+t62;
t100 = t46+t58+t60;
t78 = -t69;
t79 = -t70;
t81 = -t73;
t82 = -t75;
t94 = Y.*t93;
t95 = Z.*t92;
t101 = X.*t100;
t102 = Y.*t99;
t103 = fy.*t100;
t104 = t99.*v0;
t106 = t44+t51+t80;
t114 = t36+t74+t83;
t116 = -Y.*(-t35+t64+t69);
t117 = -Z.*(-t34+t59+t73);
t119 = -fy.*(-t35+t64+t69);
t120 = -v0.*(-t34+t59+t73);
t122 = v0.*(-t34+t59+t73);
t96 = -t94;
t97 = -t95;
t105 = t41+t49+t82;
t107 = t45+t50+t79;
t112 = t34+t71+t81;
t113 = t35+t77+t78;
t123 = t104+t119;
t108 = X.*t107;
t109 = Z.*t105;
t110 = fy.*t105;
t111 = t107.*v0;
t126 = Y.*t123;
t129 = d_y+t86+t91+t96;
t130 = d_z+t87+t90+t97;
t115 = -t108;
t118 = -t109;
t121 = -t111;
t127 = t110+t122;
t124 = t103+t121;
t128 = Z.*t127;
t132 = t8+t101+t116+t118;
t133 = t9+t102+t115+t117;
t125 = X.*t124;
t131 = -t128;
t134 = 1.0./t133;
t135 = t134.^2;
t136 = fy.*t129.*t134;
t138 = t13+t14+t125+t126+t131;
t137 = -t136;
t139 = fy.*t130.*t132.*t135;
t140 = t134.*t138;
t141 = -t140;
t143 = t137+t139+1.0;
t142 = t141+v_obs;
t144 = 1.0./t143;
error_obs_weighted = [-u_obs+t134.*(fx.*(t2+tx)-X.*(t107.*u0+fx.*(-t36+t61+t76))+Z.*(fx.*t98-u0.*(-t34+t59+t73))+t9.*u0-Y.*(fx.*t106-t99.*u0))+t144.*(fx.*t134.*(d_x-X.*(t50.*w_y+t58.*w_z)+Y.*(t56.*w_y-t35.*w_z)+Z.*(t34.*w_y+t49.*w_z))-fx.*t130.*t135.*(t2+tx-Y.*t106+Z.*t98-X.*(-t36+t61+t76))).*(t140-v_obs);t144.*(t140-v_obs)];

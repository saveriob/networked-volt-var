function q = localstatic(v,params)

	slope = (v-params.VMAX2) .* (-params.QMIN) / (params.VMAX-params.VMAX2);
	q = -min(max(slope,0),(-params.QMIN));

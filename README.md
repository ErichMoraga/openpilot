So, we know that most Toyotas have the EPS motor on the steering column, right? We also know that most modern cars have Variable Ratio Steering. So, why don't we account for this in the MPC model?

(Matlab video about MPC, check it out)
https://youtu.be/8U0xiOkDcmw

Variable Ratio Steering
![alt text](https://i.imgur.com/HIruEfd.png)

MPC needs to know a good estimate of where your car will be from a specificed output, the desired_angle. There's math involved, but MPC relies upon the steer ratio to help calculate the final tire angle. The issue is that only one steering ratio is accounted for, in settings and in the paramslearner (that slowly learns the best steer ratio over time).

This means that you end up with a steer ratio that works well in straights, or in corners, not both and as such, OP ends up working better in straights, or corners, with it usually being straights as straight roads are the most common (at least in Indiana).

I try to enhance this in OP by splitting out the center and off-center steer ratios into discrete values everywhere they appear in OP. This includes vehicle params, MPC model, ParamsLearner, etc. Hopefully with these changes, the best steer ratios can be found across the steering rack.

It would be very helpful if I, or someone could up with an easy way to measure the steer ratios around the rack at specified angles, it'd be super handy to know of those breakpoints, and the specific ratios involved without having to take apart a steering rack.

It'd also be great if I didn't hardcode the angles for the breakpoint as different models need different configs, but I'll chaulk it up to an enhancement request later and it'd involve changing yet another struct.

---
permalink: /
title: "Bio"
author_profile: true
redirect_from:
  - /about/
  - /about.html
---

I am a PhD at CMU's Robotics Institute, specializing in dexterous manipulation at the intersection of robot learning and model-based planning. My main line of work is generating high-quality plans for dexterous manipulation and using them as synthetic demonstrations to accelerate RL training. I'm passionate about continuing to bring my strong classical robotics background (planning, controls, optimization) to improving the efficiency and robustness of RL. 

Previously, I worked on learned safe control for quadrotors and multi-task RL for locomotion.
I care deeply about making algorithms work on real hardware, and I've shipped all my research on physical systems: bimanual KUKA, quadrotors, and legged millirobots. 

At CMU, I am advised by [Changliu Liu](https://www.cs.cmu.edu/~cliu6/) and [John Dolan](https://www.ri.cmu.edu/ri-faculty/john-m-dolan/), and a recipient of the Qualcomm Graduate Fellowship. I also spent a few semesters on the Dexterous Mobile Manipulation team at [Robotics and AI Institute](https://theaiinstitute.com/). Prior to CMU, I did my undergrad in EECS at UC Berkeley, where I worked with [Sergey Levine](https://people.eecs.berkeley.edu/~svlevine/) on deep RL for robotics.

You can reach me at simin.liu.1314 -at- gmail dot com

<span style="color: #c00; font-weight: bold;">I am on the job market — please reach out if you have a relevant role.</span>

---

<div class="news-section" markdown="1">

## News

- [March 2026] Talk at Duke Manipulation Seminar on [contact-rich manipulation](https://docs.google.com/presentation/d/1KOY8aUZNnUM0n5xJHm-Gswq5P9HSXQ7ksvZ_J_OuEY8/edit?usp=sharing) 
- [March 2026] Talk at CMU Manipulation Seminar on [contact-rich manipulation](https://docs.google.com/presentation/d/1KOY8aUZNnUM0n5xJHm-Gswq5P9HSXQ7ksvZ_J_OuEY8/edit?usp=sharing)
<!-- - [Fall 2025] [Passed defense!](/images/defense.jpeg) -->
- [Jan 2025] [Paper](https://arxiv.org/abs/2601.10827) submitted to IEEE T-RO. 
- [April 2025] [Paper](https://arxiv.org/abs/2408.00117) accepted at ACM Transactions on Cyber-Physical Systems 
- [Sept 2024] Started research internship at the Robotics and AI Institute, with [Tao Pang](https://pangtao.xyz/)
- [Jun 2024] [Paper](https://arxiv.org/abs/2311.00822) accepted at ECC
- [May 2023] Selected for Qualcomm Graduate Fellowship
- [April 2023] [Paper](https://arxiv.org/abs/2306.06611) accepted at ICLR
- [Sept 2022] [Paper](https://arxiv.org/abs/2211.11056) accepted at CORL

</div>

---

## Selected Projects

<div class="portfolio-list">

<div class="portfolio-group">
  <h3 class="portfolio-group-title">Dexterous, Contact-Rich Manipulation</h3>
  <p class="portfolio-group-intro">Building learning and planning algorithms for dexterous, contact-rich manipulation, where the full arm is used to move objects, not just the end-effector. Contact-rich manipulation is more challenging and more expressive than pick-and-place.</p>

  <div class="portfolio-subitem">
    <h4>Higher-Quality Model-Based Planning</h4>
    <p>We built a planner that enables a bimanual system to move large, heavy objects using whole-arm contact. Unlike prior sampling-based approaches, which could produce whole-arm plans but at poor quality, this planner globally optimizes over grasp sequencing and in-grasp motion jointly. This joint optimization produces consistent, efficient plans suitable for hardware deployment and reinforcement learning. </p>
    <div class="portfolio-media">
      <!-- <img src="/images/portfolio/safe_control_highdim.png" alt="Safe control for high-dimensional systems" /> -->
      <video autoplay loop muted playsinline preload="auto">
      <source src="/images/portfolio/crm.mp4" type="video/mp4">
      </video>
      <p class="portfolio-caption">Our method generates short, direct plans that leverage all manipulator surfaces, not just end-effectors.</p>
    </div>
  </div>

  <div class="portfolio-subitem">
    <h4>Learning from Planner-Generated Demonstrations </h4>
    <p> (Ongoing work): Synthetic data avoids the cross-embodiment transfer issues of human data, and is therefore a promising additional data source for today's VLAs and RL algorithms. Teleoperation is also often awkward for contact-rich manipulation. Building on our planner for contact-rich manipulation, we're using its outputs as synthetic demonstrations for RL, and measuring how much they accelerate training and where the gains are largest. </p>
    <div class="portfolio-media">
      <div class="portfolio-grid-3x3">
        <img src="/images/grs_query_7.gif" alt="Planner demo" />
        <img src="/images/grs_query_9.gif" alt="Planner demo" />
        <img src="/images/grs_query_63.gif" alt="Planner demo" />
        <img src="/images/grs_query_66.gif" alt="Planner demo" />
        <img src="/images/grs_query_75.gif" alt="Planner demo" />
        <img src="/images/grs_query_78.gif" alt="Planner demo" />
        <img src="/images/grs_query_115.gif" alt="Planner demo" />
        <img src="/images/grs_query_161.gif" alt="Planner demo" />
        <img src="/images/grs_query_190.gif" alt="Planner demo" />
      </div>
      <p class="portfolio-caption">A sampling of planner-generated demonstrations for different (start, goal) queries.</p>
    </div>
  </div>
</div>

<div class="portfolio-group">
  <h3 class="portfolio-group-title">Safe Control</h3>
  <p class="portfolio-group-intro">Built full-stack safe control systems for agile quadrotors, where a safety filter wraps a nominal planner or controller and intervenes only when needed.</p>

  <div class="portfolio-subitem">
    <h4>Safe Control for Uncertain Systems</h4>
    <p>Most safety filter synthesis approaches assume a known model, which is impractical.
We synthesized robust-adaptive safety filters for nonlinear systems with unknown model parameters. The filter can be combined with online parameter estimation for end-to-end safety. Generated a collision-avoidance filter for a quadrotor with unknown drag in minutes on a regular laptop CPU. </p>
    <div class="portfolio-media">
      <!-- <img src="/images/portfolio/safe_control_uncertain.png" alt="Safe control for uncertain systems" /> -->
      <video autoplay loop muted playsinline preload="auto">
      <source src="/images/portfolio/geofence_drone_4x.mp4" type="video/mp4">
      </video>
      <p class="portfolio-caption">Our safety filter keeps the drone inside the geofence despite unknown wind gusts.</p>
    </div>
  </div>

  <div class="portfolio-subitem">
    <h4>Safe Control for High-Dimensional Systems</h4>
    <p>Grid-based RL can synthesize safety filters via an optimal control formulation, but it quickly becomes intractable beyond ~6D. We take inspiration from Q-learning and nonlinear control and introduce "neural control barrier functions", a neural safety filter parameterization that scales synthesis to systems with high state dimension. We learn a pendulum-balancing filter for a 10D quadrotor-pendulum in under 2 hours, and it intervenes orders of magnitude less often than an MPC-based safety filter. </p>
    <div class="portfolio-media">
      <!-- <img src="/images/portfolio/safe_control_highdim.png" alt="Safe control for high-dimensional systems" /> -->
      <video autoplay loop muted playsinline preload="auto">
      <source src="/images/portfolio/ncbf_drone_web.mp4" type="video/mp4">
      </video>
      <p class="portfolio-caption">Our safety filter prevents the pendulum from falling while the nominal controller stabilizes the quadrotor (10D quadrotor–pendulum system).</p>
    </div>
  </div>
</div>

<div class="portfolio-item">
  <h3>Multitask RL for Adaptive Locomotion</h3>
  <p>Model-based methods and standard RL both struggle to generalize locomotion controllers to previously unseen disturbances. We develop a multitask model-based RL algorithm that trains an adaptable dynamics model on a few hours of domain-randomized data — scenarios like leg loss, terrain variation, and payload changes. We demonstrate a 3–8x increase in path-following reward over a no-adaptation baseline on unseen disturbances.</p>
  <div class="portfolio-media">
    <!-- <img src="/images/portfolio/locomotion.png" alt="Locomotion under disturbances" /> -->
    <video autoplay loop muted playsinline preload="auto">
    <source src="/images/portfolio/maml_2x.mp4" type="video/mp4">
    </video>
    <p class="portfolio-caption">The robot closely tracks the path despite leg loss, terrain changes, payload variation, and state-estimation error.</p>
  </div>
</div>

</div>

<style>
.news-section ul {
  font-size: 0.9rem;
}
.portfolio-list {
  display: flex;
  flex-direction: column;
  gap: 3rem;
  margin-top: 1rem;
}
.portfolio-item h3 {
  margin-top: 0;
  margin-bottom: 0.5rem;
}
.portfolio-media {
  margin-top: 1rem;
  background: #f5f5f5;
  border-radius: 6px;
  overflow: hidden;
}
.portfolio-media img,
.portfolio-media video {
  width: 100%;
  display: block;
}
.portfolio-grid-3x3 {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 4px;
}
.portfolio-grid-3x3 img {
  width: 100%;
  height: auto;
  display: block;
  object-fit: cover;
}
.portfolio-caption {
  margin: 0;
  padding: 0.6rem 0.9rem;
  font-size: 0.875rem;
  color: #555;
  font-style: italic;
}
.portfolio-group {
}
.portfolio-group-title {
  margin-top: 0;
  margin-bottom: 0.25rem;
}
.portfolio-group-intro {
  margin-bottom: 0;
  opacity: 0.85;
}
.portfolio-subitem {
  margin-top: 1.5rem;
  border-left: 3px solid currentColor;
  padding-left: 1.25rem;
}
.portfolio-subitem h4 {
  margin-top: 0;
  margin-bottom: 0.4rem;
  font-size: 1rem;
}
</style>

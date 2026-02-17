---
permalink: /
title: "Simin Liu"
author_profile: true
redirect_from:
  - /about/
  - /about.html
---

I'm a recent PhD graduate of CMU's Robotics Institute. My research combines machine learning and model-based planning and control for structured, scalable autonomy. In the past, I've worked on planning for contact-rich manipulation, safe control for agile aerial robots, and control for locomotion under disturbances. At CMU, I was honored to be advised by [Changliu Liu](https://www.cs.cmu.edu/~cliu6/) and [John Dolan](https://www.ri.cmu.edu/ri-faculty/john-m-dolan/) and supported by the Qualcomm Graduate Fellowship. I have also spent time at the [Robotics and AI Institute](https://theaiinstitute.com/) (previously the Boston Dynamics AI Institute).

Prior to CMU, I did my undergrad at UC Berkeley in EECS and Math, where I worked with [Sergey Levine](https://people.eecs.berkeley.edu/~svlevine/) on deep RL for robotics.

<span style="color: #c00; font-weight: bold;">I am on the job market — please reach out if you have a relevant role!</span>

---

## News

- [Feb 2026] Talk at CMU Manipulation Seminar: "Global Planning for Contact-Rich Manipulation"
- [2025] Paper accepted at ACM Transactions on Cyber-Physical Systems: certifying robustness of learned perception
- [Fall 2025] Successfully passed defense
- [Sep 2024–May 2025] Research internship at the Robotics and AI Institute (formerly Boston Dynamics AI Institute)
- [Jun 2024] Paper at European Controls Conference: safe control for uncertain systems
- [2023] Qualcomm Graduate Fellowship (18 selected from 182 applicants)
- [May 2023] Paper at ICLR 2023 (Oral, top 25%)

---

## Research

<div class="portfolio-list">

<div class="portfolio-item">
  <h3>High-Performance Planning for Contact-Rich Manipulation</h3>
  <p>Sampling-based planners for contact-rich manipulation are common, but they produce circuitous, inefficient trajectories. Improving beyond these methods is hard because the action space is combinatorial and cannot be exhaustively searched. Our insight is to reduce the action space to higher-level, algorithmically-generated reachable set primitives, enabling optimal search in this space in under a minute for bimanual manipulation. </p>
  <div class="portfolio-media">
    <!-- Replace src with your image or embed a video here -->
    <img src="/images/portfolio/contact_planning.png" alt="Contact-rich manipulation planning" />
    <p class="portfolio-caption">Our method generates shorter, more direct plans than a state-of-the-art sampling-based planner.</p>
  </div>
</div>

<div class="portfolio-group">
  <h3 class="portfolio-group-title">Safe Control</h3>
  <p class="portfolio-group-intro">We build reactive safety filters that wrap a nominal controller, modifying its commands only when safety is at risk. A good filter is minimally invasive while respecting input bounds and system dynamics that limit how quickly safety maneuvers can be executed.</p>

  <div class="portfolio-subitem">
    <h4>Safe Control for Uncertain Systems</h4>
    <p>Most safety filter synthesis approaches assume a known model, which is impractical. We consider systems with uncertain model parameters and devise a sum-of-squares programming algorithm for synthesis. We generate a geofencing (stay-within-region) safety filter for a drone with unknown drag in minutes on a regular laptop CPU.</p>
    <div class="portfolio-media">
      <img src="/images/portfolio/safe_control_uncertain.png" alt="Safe control for uncertain systems" />
      <p class="portfolio-caption">Our safety filter keeps the drone inside the geofence despite unknown wind gusts.</p>
    </div>
  </div>

  <div class="portfolio-subitem">
    <h4>Safe Control for High-Dimensional Systems</h4>
    <p>Grid-based RL can synthesize safety filters via an optimal control formulation, but it quickly becomes intractable beyond ~6D. We take inspiration from deep RL and nonlinear control, posing this problem as training a neural function to satisfy control barrier function (CBF) conditions. We synthesize a safety filter for a 10D system with &lt;2 hours of training, and it triggers orders of magnitude less often than model predictive control (MPC).</p>
    <div class="portfolio-media">
      <img src="/images/portfolio/safe_control_highdim.png" alt="Safe control for high-dimensional systems" />
      <p class="portfolio-caption">Our safety filter prevents the pendulum from falling while the nominal controller stabilizes the quadrotor (10D quadrotor–pendulum system).</p>
    </div>
  </div>
</div>

<div class="portfolio-item">
  <h3>Model-Based RL for Locomotion Under Disturbances</h3>
  <p>We study adaptive locomotion under a broad range of previously unseen disturbances (external forces, state-estimation error, and unmodeled effects), where both purely model-based methods and standard RL can struggle to generalize. We combine adaptive control with meta-learning, performing online model estimation on a neural dynamics model and applying the model inside a sampling-based controller. We pre-train  dynamics features offline using 1–2 hours of disturbance data, and at deployment we find the controller can track a path closely despite unseen disturbances.</p>
  <div class="portfolio-media">
    <img src="/images/portfolio/locomotion.png" alt="Locomotion under disturbances" />
    <p class="portfolio-caption">The robot closely tracks the path despite leg loss, terrain changes, payload variation, and state-estimation error.</p>
  </div>
</div>

</div>

<style>
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
.portfolio-media img {
  width: 100%;
  display: block;
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

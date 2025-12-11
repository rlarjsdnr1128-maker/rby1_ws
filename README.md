* Behavior Robot Suite (BRS) with Diffusion Steering via Reinforcement Learning (DSRL)

-Abstact-

Achieving natural whole-body motion in mobile manipulators is essential for real-world development, yet remains challenging due to their complex hardware and the limitations of existing learning models. This study proposes a BRS+DSRL framework that integrates Behavior Robot Suite (BRS) with Diffusion Steering via Reinforcement Learning (DSRL) to achieve natural whole-body motion learning for mobile manipulator robots. We first construct a data validation pipeline to verify the multimodal inputs required for training, which consist of point clouds, joint commands, and base/gripper states. Then we develop a VR teleoperation-based data collection pipeline that enables efficient acquisition of demonstration data.
Before applying the BRS+DSRL framework to the mobile manipulator, we first evaluated BRS in simulation using BiGym to examine its stability. In this process, we identified fundamental limitations of BRS. BRS exhibits unstable learning behavior driven by its hierarchical structure and strong dependence on high-quality data whereas a non-hierarchical structure such as ACT learns stably under identical conditions. These findings indicate that BRS is not well suited for practical whole-body manipulation learning. Instead, our attempt to integrate DSRL into BRS suggests a promising direction for future work: applying DSRL to alternative policy architectures. Furthermore, the data collection and validation pipelines developed in this study establish a foundation for future research on integrating DSRL with more effective robot learning models.

-Video-

https://drive.google.com/drive/folders/14Dlog4Vl4KzhKTngIa-goFCKcgvSO8dp?usp=sharing

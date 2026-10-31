# Recorded numerical outputs

`experiment_report.json` identifies each run, its solver history, acceptance flag, and validation metrics. The trajectory CSV files include failed candidates for diagnosis. **Only entries with `success: true` are accepted plans.** Columns use SI units and the same conventions in Chapters 5 and 6.

Chapter 5 keeps the failed original eight-vehicle run separate from its initializer-recovery experiment. Chapter 6 includes final fleet-wide time-dilation factors and repaired completion times. Figure generation uses the saved, accepted demonstration; it does not create synthetic experimental data.

`environment.json` records software, numerical-thread configuration, and timing limitations. Re-running the MATLAB entry points writes new results to `output` rather than overwriting this recorded report.

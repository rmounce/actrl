Refactoring goals:
- Create a native Home Assistant component
- Live tuning of parameters

- Existing entities:
    - Main A/C climate component
        - Optional: Follow me?
    - For each zone:
        - Damper (cover)
        - Temperature sensor

- New virtual entities:
    - Climate component per zone

Internal changes
- Easier to think about units
    - PID: 0 to 1 rather than 1 to -1
        0: No demand
        1: Max demand
    

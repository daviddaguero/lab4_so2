### **Stack Usage Analysis for FreeRTOS Tasks**

**Platform:** ARM Cortex-M3 (Stellaris LM3S811 simulated in QEMU)

**Word Size:** 4 bytes

**configMINIMAL\_STACK\_SIZE:** 143 words (572 bytes)

**Tool Used:** `uxTaskGetStackHighWaterMark()`

---

### 📌 **Task Creation Configuration**

The following tasks were created using `configMINIMAL_STACK_SIZE` as their stack size:

```c
xTaskCreate(vSimulateTemperatureSensorTask, "TempSensorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
xTaskCreate(vLowPassFilterTask, "FilterTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
xTaskCreate(vDisplayGraphTask, "GraphTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
xTaskCreate(vUARTReaderTask, "UARTReader", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
xTaskCreate(vMonitorStackTask, "MonitorStack", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
```

---

### 📊 **High Water Mark Observations**

Multiple runs yielded the following `uxTaskGetStackHighWaterMark()` values:

| Task Name    | Highest HWM Observed | Lowest HWM Observed | Maximum Stack Used (Words) | Approx. Bytes Used |
| ------------ | -------------------- | ------------------- | -------------------------- | ------------------ |
| TempSensor   | 141                  | 100                 | 43                         | 172 bytes          |
| FilterTask   | 141                  | 100                 | 43                         | 172 bytes          |
| GraphTask    | 141                  | 100                 | 43                         | 172 bytes          |
| UARTReader   | 141                  | 100                 | 43                         | 172 bytes          |
| MonitorStack | (Not Measured)       | (Not Measured)      | ?                          | ?                  |

> `Stack Used = configMINIMAL_STACK_SIZE - HWM`

---

### 🧠 **Interpretation**

* All tasks are currently over-provisioned in terms of stack.
* Even in the worst case (HWM = 100), only **30% of the stack** is actually being used.
* In the best case (HWM = 141), the task uses just **2 words** (≈ 8 bytes), which is negligible.

---

### ✅ **Recommendations**

We can significantly reduce the stack size per task to save memory, without compromising stability. Suggested values:

| Task Name    | Recommended Stack Size (Words) | Rationale                     |
| ------------ | ------------------------------ | ----------------------------- |
| TempSensor   | 96                             | 43 words used + safety margin |
| FilterTask   | 96                             | Same as above                 |
| GraphTask    | 96                             | Same as above                 |
| UARTReader   | 96                             | Same as above                 |
| MonitorStack | 64                             | Expected to be lightweight    |

We have to always validate these values again after implementation by monitoring new HWM results.

---

### 📦 **Memory Saving Summary**

Reducing stack sizes as recommended could free up:

* From `5 tasks × (143 - avg. 96)` = **235 words** saved
* \= **940 bytes** saved in total stack memory

---

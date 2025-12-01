# dashboard.py
import streamlit as st
import requests
import pandas as pd
import time

st.title("ESP32 Real-Time Dashboard")

# The URL where FastAPI is running locally
API_URL = "http://0.0.0.0:8000/api/data"

placeholder = st.empty()

while True:
    try:
        response = requests.get(API_URL)
        if response.status_code == 200:
            data = response.json()
            
            if data:
                df = pd.DataFrame(data)
                
                with placeholder.container():
                    # Create columns for metrics
                    kpi1, kpi2 = st.columns(2)
                    
                    # Display latest reading
                    last_reading = data[-1]
                    kpi1.metric(label="Temperature (°C)", value=last_reading['temperature'])
                    kpi2.metric(label="Humidity (%)", value=last_reading['humidity'])
                    
                    # Display Chart
                    st.line_chart(df[['temperature', 'humidity']])
                    st.dataframe(df)
            else:
                with placeholder.container():
                    st.warning("Waiting for data...")
        
    except Exception as e:
        st.error(f"Error connecting to API: {e}")

    # Refresh every 2 seconds
    time.sleep(2)
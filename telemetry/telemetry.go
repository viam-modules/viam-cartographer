package telemetry

import (
	"time"

	"go.viam.com/utils/perf"
)

func SetupTelemetry() (perf.Exporter, error) {
	exporter := perf.NewDevelopmentExporterWithOptions(perf.DevelopmentExporterOptions{
		ReportingInterval: time.Second, // Good reporting interval time?
	})
	if err := exporter.Start(); err != nil {
		return nil, err
	}

	return exporter, nil
}

// Package telemetry provides setup for logging through telementry
package telemetry

import (
	"time"

	"go.viam.com/utils/perf"
)

// Init sets up telemetry so logs and stats can be reported.
func Init() (perf.Exporter, error) {
	exporter := perf.NewDevelopmentExporterWithOptions(perf.DevelopmentExporterOptions{
		ReportingInterval: time.Second, // Good reporting interval time?
	})
	if err := exporter.Start(); err != nil {
		return nil, err
	}

	return exporter, nil
}

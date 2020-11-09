#ifndef SEMANTICPOINTCLOUDMETRIC_H
#define SEMANTICPOINTCLOUDMETRIC_H

#include "metrics/Metric.h"
#include "outputs/Output.h"

namespace slambench {
    namespace metrics {

        using slambench::values::SemanticPointCloudValue;

        class SemanticPointCloudMetric : public Metric {
            private:
                const slambench::outputs::BaseOutput * const tested;
                const slambench::outputs::BaseOutput * const gt;

                slambench::TimeStamp lastMeasurement;

            public:
                SemanticPointCloudMetric(const slambench::outputs::BaseOutput * const tested,
                                         const slambench::outputs::BaseOutput * const gt);

                ~SemanticPointCloudMetric() = default;

                const slambench::values::ValueDescription& GetValueDescription() const override;
                const std::string& GetDescription() const override;

                void MeasureStart(Phase* phase) override;
                void MeasureEnd(Phase* phase) override;

                Value *GetValue(Phase* phase) override;
        };
    }
}

#endif
